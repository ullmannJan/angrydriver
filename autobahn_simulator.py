# autobahn_bmw_driver_view.py
import sys
import math
import random
import pygame
import enum
import re
import serial
import threading
import queue
import os
# ---------- Config ----------
W, H = 900, 600  # Original design dimensions (will be updated to fullscreen)
FPS = 60
# ROAD_BOTTOM_WIDTH moved to dynamic calculation after fullscreen init
ROAD_TOP_HALF_WIDTH = 12            # small but not needle-thin
HORIZON_BASE = int(H * 0.50)        # eye level (will be recalculated after fullscreen)
LANES = 5
MAX_KMH = 300
ACCEL_KMH_S = 35                    # increased from 10 for better overtaking ability
AUTO_ACCEL_KMH_S = 12               # automatic acceleration rate (slower than manual)
BRAKE_KMH_S = 161
DRAG_KMH_S = 8                      # reduced from 15 to maintain higher speeds easier
PX_PER_KMH = 2.0                    # still used for dash motion only
LANE_CHANGE_SPEED = 650
DASH_SPACING = 60
DASH_BASE_LEN = 28
DASH_BASE_W = 6
# Car spawning & projection (world-space meters)
CAR_SPAWN_RATE = 0.4               # cars per second (reduced for easier overtaking)
CAR_BASE_WIDTH = 160              # pixels at z -> 0 (75% of lane width: 320px * 0.75 = 240px)
CAR_BASE_HEIGHT = 320             # proportionally scaled height (2:1 ratio for typical car)
CAR_MIN_SPEED_KMH = 70              # reduced from 80 to make some cars slower
CAR_MAX_SPEED_KMH = 140             # reduced from 160 to make fast cars more catchable
CAR_COLOR = (0, 0, 0)
Z_H = 180.0                         # perspective scale [m]; ~distance where size is half
SPAWN_Z_MIN = 140.0                 # min spawn distance ahead [m]
SPAWN_Z_MAX = 900.0                 # max spawn distance ahead [m]
DESPAWN_AHEAD_Z = 2200.0            # if it gets this far away, cull
DESPAWN_BEHIND_Z = -30.0            # once it passes the camera by this much, cull
# Driver-view tuning
DRIVER_OFFSET_IN_LANE = -0.18       # fraction of lane width; negative = sit left
YAW_FROM_STEER = 0.015              # px of VP shift per px/s lateral velocity
PITCH_FROM_ACCEL = -0.9             # px horizon moves per m/s^2 accel (negative = drops on accel)
YAW_SMOOTH_HZ = 8.0
PITCH_SMOOTH_HZ = 6.0
BG = (20, 20, 22)
ROAD = (40, 40, 42)
EDGE = (180, 180, 180)
DASH = (220, 220, 220)
HUD = (240, 240, 240)
HOOD = (15, 15, 18)
# ---------- Helpers ----------
def clamp(x, lo, hi):
   return lo if x < lo else hi if x > hi else x
def lerp(a, b, t): return a + (b - a) * t
def kmh_to_mps(v_kmh): return v_kmh * (1000.0 / 3600.0)

# ---------- Serial Control System ----------
# Configure your serial port
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"  # Default to COM15 if no argument is provided
BAUD_RATE = 115200

class Control(enum.Enum):
    SwipeLeft = 1
    SwipeRight = 2
    SwipeUp = 3
    SwipeDown = 4
    Push = 5

def read_serial_continuously(control_queue, serial_port, baud_rate):
    """Continuously read from serial port in a separate thread."""
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            print(f"Connected to {serial_port} at {baud_rate} baud")
            while True:
                try:
                    # Read a line from the serial port
                    line = ser.readline().decode("utf-8").strip()
                    if not line:
                        continue
                    
                    # Print raw COM command received
                    print(f"Raw COM command received: '{line}'")
                    
                    # Use a regular expression to extract gestures
                    match = re.search(r'\.(\w+)', line)
                    if match:
                        gesture = match.group(1)
                        print(f"Parsed gesture: {gesture}")
                        
                        if gesture == "SwipeLeft":
                            control_queue.put(Control.SwipeLeft)
                            print("→ Action: Lane change LEFT")
                        elif gesture == "SwipeRight":
                            control_queue.put(Control.SwipeRight)
                            print("→ Action: Lane change RIGHT")
                        elif gesture == "SwipeUp":
                            control_queue.put(Control.SwipeUp)
                            print("→ Action: Accelerate")
                        elif gesture == "SwipeDown":
                            control_queue.put(Control.SwipeDown)
                            print("→ Action: Brake")
                        elif gesture == "Push":
                            control_queue.put(Control.Push)
                            print("→ Action: LICHTHUPE/Horn")
                        else:
                            print(f"→ Unknown gesture: {gesture}")
                    else:
                        print(f"→ No gesture pattern found in: '{line}'")
                        
                except Exception as e:
                    print(f"Serial read error: {e}")
                    continue
                    
    except Exception as e:
        print(f"Serial connection error: {e}")
        print("Continuing without serial input...")

def get_serial_control(control_queue):
    """Get the next control from the queue, or None if empty."""
    try:
        return control_queue.get_nowait()
    except queue.Empty:
        return None
# Perspective lerp anchored at the (dynamic) horizon.
def lane_x_at_y(y, f, vp_x, left_bottom_x, right_bottom_x, horizon_y):
   # 0 at horizon, 1 at bottom
   t = clamp((y - horizon_y) / (H - horizon_y), 0.0, 1.0)
   x_bottom = left_bottom_x + (right_bottom_x - left_bottom_x) * f
   return vp_x + (x_bottom - vp_x) * t
def project_y_from_z(z_m, horizon_y):
   """
   Map world distance (meters ahead of camera) to screen y.
   t is the perspective scale used consistently for size/width.
   z -> infinity => y -> horizon; z -> 0 => y -> bottom.
   """
   t = Z_H / (z_m + Z_H)
   y = horizon_y + (H - horizon_y) * t
   return y, t
# ---------- Car System ----------
class Car:
   def __init__(self, lane, z_m, speed_kmh):
       self.lane = lane                 # 0..LANES-1
       self.target_lane = lane          # Lane the car is moving towards
       self.lane_change_progress = 0.0  # 0.0 = at original lane, 1.0 = at target lane
       self.lane_change_speed = 2.0     # How fast cars change lanes (progress per second)
       self.z_m = z_m                   # world distance ahead of player [m]
       self.speed_kmh = speed_kmh       # car's own speed [km/h]
       self.image_index = random.randint(0, 2)  # Random car image (0, 1, or 2)
       
   def start_lane_change(self, new_target_lane):
       """Start changing to a new lane."""
       if new_target_lane != self.target_lane and 0 <= new_target_lane < LANES:
           self.lane = self.target_lane  # Current position becomes the starting lane
           self.target_lane = new_target_lane
           self.lane_change_progress = 0.0
           
   def get_current_lane_position(self):
       """Get the current lane position (can be fractional during lane changes)."""
       if self.lane_change_progress >= 1.0:
           return self.target_lane
       return self.lane + (self.target_lane - self.lane) * self.lane_change_progress
       
   def update(self, dt, player_speed_kmh):
       # Relative motion strictly in world space (meters)
       rel_mps = kmh_to_mps(self.speed_kmh - player_speed_kmh)
       self.z_m += rel_mps * dt
       
       # Update lane changing
       if self.lane_change_progress < 1.0:
           self.lane_change_progress += self.lane_change_speed * dt
           if self.lane_change_progress >= 1.0:
               self.lane_change_progress = 1.0
               self.lane = self.target_lane  # Lane change complete
   def is_offscreen(self):
       return (self.z_m > DESPAWN_AHEAD_Z) or (self.z_m < DESPAWN_BEHIND_Z)
   def get_screen_rect(self, vp_x, left_bottom_x, right_bottom_x, horizon_y):
       # If effectively "at infinity", skip
       if self.z_m <= 0.0:
           # passed the camera; draw only until we cull shortly after
           pass
       y, t = project_y_from_z(max(self.z_m, 0.01), horizon_y)
       if y <= horizon_y or y > H + 200:
           return None
       # Lane center position using same t-driven geometry (with animated lane changing)
       current_lane_pos = self.get_current_lane_position()
       lane_center_f = (current_lane_pos + 0.5) / LANES
       car_x = lane_x_at_y(y, lane_center_f, vp_x, left_bottom_x, right_bottom_x, horizon_y)
       # Scale car dimensions by perspective t
       scale = clamp(t, 0.08, 1.0)
       car_width = max(4, int(CAR_BASE_WIDTH * scale))
       car_height = max(6, int(CAR_BASE_HEIGHT * scale))
       return pygame.Rect(
           int(car_x - car_width // 2),
           int(y - car_height // 2),
           car_width,
           car_height
       )
def spawn_car():
   """Spawn a new car in a random lane at a reasonable distance ahead."""
   lane = random.randint(0, LANES - 1)
   speed = random.uniform(CAR_MIN_SPEED_KMH, CAR_MAX_SPEED_KMH)
   z_m = random.uniform(SPAWN_Z_MIN, SPAWN_Z_MAX)
   return Car(lane, z_m, speed)

def is_car_too_close_ahead(cars, player_lane, min_distance=25.0):
   """Check if there's a car too close ahead in the player's lane."""
   for car in cars:
       # Check if car is in the same lane as player (considering animated lane changes)
       current_lane_pos = car.get_current_lane_position()
       if abs(current_lane_pos - player_lane) < 0.5:  # Within half a lane width
           # Check if car is ahead and within minimum distance
           if 0 < car.z_m < min_distance:
               return True
   return False

def get_closest_car_ahead(cars, player_lane):
   """Get the closest car ahead in the player's lane, or None if no car ahead."""
   closest_car = None
   closest_distance = float('inf')

   for car in cars:
       current_lane_pos = car.get_current_lane_position()
       if abs(current_lane_pos - player_lane) < 0.5 and car.z_m > 0:  # Car is ahead in same lane
           if car.z_m < closest_distance:
               closest_distance = car.z_m
               closest_car = car

   # Always return a tuple of (car, distance)
   if closest_car is not None:
       return closest_car, closest_distance
   else:
       return None, float('inf')

def try_make_car_switch_lanes(cars, player_lane, success_chance=0.4):
   """Try to make the nearest car in the player's lane switch lanes (stochastic)."""
   # Find the nearest car in the player's lane
   nearest_car = None
   nearest_distance = float('inf')
   
   for car in cars:
       current_lane_pos = car.get_current_lane_position()
       if abs(current_lane_pos - player_lane) < 0.5 and car.z_m > 0:  # Car ahead in same lane
           if car.z_m < nearest_distance:
               nearest_distance = car.z_m
               nearest_car = car
   
   if nearest_car is None:
       return False  # No car to move
   
   # Stochastic check - car might not respond
   if random.random() > success_chance:
       return False  # Car didn't respond to honking/flashing
   
   # Find a valid lane to move to (prefer right lane, then left)
   possible_lanes = []
   current_lane = int(round(nearest_car.get_current_lane_position()))
   
   # Check right lane first (passing lane etiquette)
   if current_lane + 1 < LANES:
       possible_lanes.append(current_lane + 1)
   
   # Then check left lane
   if current_lane - 1 >= 0:
       possible_lanes.append(current_lane - 1)
   
   if not possible_lanes:
       return False  # No lanes available
   
   # Choose a random lane from available options
   new_lane = random.choice(possible_lanes)
   nearest_car.start_lane_change(new_lane)
   return True  # Successfully initiated lane change

def draw_car_with_image(screen, car, car_rect, car_images):
    """Draw a car using its assigned image, scaled to the correct size."""
    if not car_rect or not car_images:
        return
    
    # Get the car's image
    original_img = car_images[car.image_index]
    
    # Scale the image to fit the calculated rect
    scaled_img = pygame.transform.scale(original_img, (car_rect.width, car_rect.height))
    
    # Blit the scaled image
    screen.blit(scaled_img, car_rect.topleft)

def play_horn_sound(horn_sound):
    """Play the horn sound if available."""
    if horn_sound:
        try:
            horn_sound.play()
        except pygame.error as e:
            print(f"Could not play horn sound: {e}")

def draw_cockpit(screen, cockpit_img):
    """Draw the cockpit image at the bottom of the screen at full screen width."""
    if cockpit_img:
        # Always use full screen width
        target_width = W
        cockpit_rect = cockpit_img.get_rect()
        target_height = int(cockpit_rect.height * (target_width / cockpit_rect.width))
        
        # Limit height to reasonable maximum (40% of screen height for better first-person view)
        max_height = int(H * 0.4)
        if target_height > max_height:
            target_height = max_height
            # Keep full width even if we have to stretch the image slightly
        
        scaled_cockpit = pygame.transform.scale(cockpit_img, (target_width, target_height))
        
        # Position at bottom, full width
        cockpit_x = 0  # Start at left edge for full width
        cockpit_y = H - target_height
        screen.blit(scaled_cockpit, (cockpit_x, cockpit_y))
        return target_height  # Return height for other elements to avoid overlap
    return 115  # Return default hood height if no cockpit image

def make_dash_rows(spacing, start_y):
   ys = []
   y = start_y - spacing
   while y < H + spacing:
       ys.append(y)
       y += spacing
   return ys
# ---------- Init ----------
pygame.init()
pygame.mixer.init()  # Initialize mixer for sound

# Get fullscreen resolution
info = pygame.display.Info()
FULLSCREEN_W, FULLSCREEN_H = info.current_w, info.current_h

# Set up fullscreen display
screen = pygame.display.set_mode((FULLSCREEN_W, FULLSCREEN_H), pygame.FULLSCREEN)
pygame.display.set_caption("BMW Autobahn - Driver View")

# Update W and H to use fullscreen dimensions
W, H = FULLSCREEN_W, FULLSCREEN_H

# Calculate scaling factors based on original design (900x600)
ORIGINAL_W, ORIGINAL_H = 900, 600
SCALE_X = W / ORIGINAL_W
SCALE_Y = H / ORIGINAL_H

# Scale road width to match new screen width
ROAD_BOTTOM_WIDTH = int(1600 * SCALE_X)

# Recalculate horizon for new screen height
HORIZON_BASE = int(H * 0.50)

# Scale other pixel-based values
LANE_CHANGE_SPEED = int(650 * SCALE_X)  # Scale lane change speed
CAR_BASE_WIDTH = int(CAR_BASE_WIDTH * SCALE_X)  # Scale car width
CAR_BASE_HEIGHT = int(CAR_BASE_HEIGHT * SCALE_Y)  # Scale car height
DASH_SPACING = int(60 * SCALE_Y)  # Scale dash spacing
DASH_BASE_LEN = int(28 * SCALE_Y)  # Scale dash length
DASH_BASE_W = int(6 * SCALE_X)  # Scale dash width

print(f"Running in fullscreen mode: {W}x{H}")
print(f"Scaling factors: X={SCALE_X:.2f}, Y={SCALE_Y:.2f}")
print(f"Scaled road width: {ROAD_BOTTOM_WIDTH}")
print(f"Horizon at: {HORIZON_BASE}")
print(f"Scaled car dimensions: {CAR_BASE_WIDTH}x{CAR_BASE_HEIGHT}")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, int(28 * min(SCALE_X, SCALE_Y)))  # Scale font size

# Load assets
def load_assets():
    """Load all game assets (images and sounds)."""
    assets = {}
    
    # Load car images
    car_images = []
    for i in range(1, 4):  # car_1.png, car_2.png, car_3.png
        try:
            car_img = pygame.image.load(f"assets/cars/car_{i}.png").convert_alpha()
            car_images.append(car_img)
        except pygame.error as e:
            print(f"Could not load car_{i}.png: {e}")
            # Create a fallback colored rectangle
            fallback = pygame.Surface((CAR_BASE_WIDTH, CAR_BASE_HEIGHT))
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # Red, Green, Blue
            fallback.fill(colors[i-1] if i-1 < len(colors) else (128, 128, 128))
            car_images.append(fallback)
    
    assets['car_images'] = car_images
    
    # Load cockpit image
    try:
        cockpit_img = pygame.image.load("assets/cockpit.png").convert_alpha()
        assets['cockpit'] = cockpit_img
    except pygame.error as e:
        print(f"Could not load cockpit.png: {e}")
        assets['cockpit'] = None
    
    # Load horn sound
    try:
        horn_sound = pygame.mixer.Sound("assets/horn.wav")
        assets['horn_sound'] = horn_sound
    except pygame.error as e:
        print(f"Could not load horn.wav: {e}")
        assets['horn_sound'] = None
    
    return assets

# Load all assets
game_assets = load_assets()

# Setup serial control system
control_queue = queue.Queue()
serial_thread = None

# Try to start serial communication in a separate thread
try:
    serial_thread = threading.Thread(
        target=read_serial_continuously, 
        args=(control_queue, SERIAL_PORT, BAUD_RATE),
        daemon=True
    )
    serial_thread.start()
    print("Serial control system started")
except Exception as e:
    print(f"Could not start serial control: {e}")
    print("Continuing with keyboard controls only")
# Player state
speed_kmh = 0.0
prev_speed_kmh = 0.0
center_lane = (LANES - 1) / 2.0
target_lane = LANES - 1  # Start in rightmost lane instead of center
curr_lane_offset_px = 0.0
prev_lane_offset_px = 0.0
lane_width_bottom = ROAD_BOTTOM_WIDTH / LANES
driver_offset_px = DRIVER_OFFSET_IN_LANE * lane_width_bottom
# Camera dynamics
horizon_y = HORIZON_BASE
yaw_px = 0.0
# Precompute dash rows (roughly aligned to the base horizon)
dash_rows = make_dash_rows(DASH_SPACING, HORIZON_BASE)
# Car management
cars = []
car_spawn_timer = 0.0
horn_feedback_timer = 0.0  # Timer for showing horn feedback
horn_feedback_text = ""
lichthupe_flash_timer = 0.0  # Timer for LICHTHUPE flash effect
running = True
while running:
   dt = clock.tick(FPS) / 1000.0
   dt_safe = max(dt, 1e-4)
   # ---------- Input ----------
   # Handle keyboard events
   for e in pygame.event.get():
       if e.type == pygame.QUIT:
           running = False
       elif e.type == pygame.KEYDOWN:
           if e.key == pygame.K_ESCAPE:
               running = False
           elif e.key in (pygame.K_LEFT, pygame.K_a):
               target_lane = max(0, target_lane - 1)
           elif e.key in (pygame.K_RIGHT, pygame.K_d):
               target_lane = min(LANES - 1, target_lane + 1)
           elif e.key in (pygame.K_h, pygame.K_SPACE):  # H for horn or SPACE for flashing lights
               # Try to make the nearest car in current lane switch lanes
               success = try_make_car_switch_lanes(cars, target_lane)
               
               # Play horn sound
               play_horn_sound(game_assets.get('horn_sound'))
               
               # Trigger LICHTHUPE flash effect when using Space
               if e.key == pygame.K_SPACE:
                   lichthupe_flash_timer = 0.3  # Flash for 0.3 seconds
               
               if success:
                   horn_feedback_text = "Car moved out of the way!"
                   horn_feedback_timer = 2.0  # Show for 2 seconds
               else:
                   horn_feedback_text = "No car responded..."
                   horn_feedback_timer = 1.5  # Show for 1.5 seconds
   
   # Handle serial controls
   serial_control = get_serial_control(control_queue)
   manual_accel = False
   manual_brake = False
   
   if serial_control:
       if serial_control == Control.SwipeLeft:
           old_lane = target_lane
           target_lane = max(0, target_lane - 1)
           print(f"GAME: SwipeLeft executed - Lane change from {old_lane + 1} to {target_lane + 1}")
       elif serial_control == Control.SwipeRight:
           old_lane = target_lane
           target_lane = min(LANES - 1, target_lane + 1)
           print(f"GAME: SwipeRight executed - Lane change from {old_lane + 1} to {target_lane + 1}")
       elif serial_control == Control.SwipeUp:
           manual_accel = True
           print("GAME: SwipeUp executed - Manual acceleration")
       elif serial_control == Control.SwipeDown:
           manual_brake = True
           print("GAME: SwipeDown executed - Manual braking")
       elif serial_control == Control.Push:
           # Push gesture triggers LICHTHUPE
           print("GAME: Push executed - Triggering LICHTHUPE/Horn")
           success = try_make_car_switch_lanes(cars, target_lane)
           
           # Play horn sound
           play_horn_sound(game_assets.get('horn_sound'))
           
           lichthupe_flash_timer = 0.3  # Flash for 0.3 seconds
           
           if success:
               horn_feedback_text = "Car moved out of the way!"
               horn_feedback_timer = 2.0  # Show for 2 seconds
               print("GAME: LICHTHUPE successful - Car moved!")
           else:
               horn_feedback_text = "No car responded..."
               horn_feedback_timer = 1.5  # Show for 1.5 seconds
               print("GAME: LICHTHUPE failed - No car responded")
   keys = pygame.key.get_pressed()
   
   # Check for cars ahead and get collision response
   car_blocking = is_car_too_close_ahead(cars, target_lane)
   closest_car, closest_distance = get_closest_car_ahead(cars, target_lane)
   
   # Calculate target speed based on traffic ahead
   target_speed = MAX_KMH  # Default to max speed when road is clear
   if closest_car and closest_distance < 50.0:  # Within 50m of a car ahead
       # Calculate safe following speed based on distance
       safety_factor = max(0.3, closest_distance / 50.0)  # 30% to 100% of car's speed
       max_safe_speed = closest_car.speed_kmh * safety_factor
       target_speed = max_safe_speed
   
   if keys[pygame.K_UP] or keys[pygame.K_w] or manual_accel:
       # Manual acceleration (faster) - keyboard or swipe up
       if not car_blocking:  # Only accelerate if no car is too close ahead
           speed_kmh += ACCEL_KMH_S * dt
       # If blocked, maintain current speed (no acceleration or deceleration)
   elif keys[pygame.K_DOWN] or keys[pygame.K_s] or manual_brake:
       # Manual braking - keyboard or swipe down
       speed_kmh -= BRAKE_KMH_S * dt
   else:
       # Automatic acceleration when no input (slower than manual)
       if not car_blocking and speed_kmh < MAX_KMH:
           speed_kmh += AUTO_ACCEL_KMH_S * dt
       else:
           # Apply drag when at max speed or blocked
           speed_kmh -= DRAG_KMH_S * dt
   
   # Apply collision response - force speed adjustment if too close to a car
   if closest_car and closest_distance < 50.0 and speed_kmh > target_speed:
       # Emergency braking when too close
       emergency_brake_rate = BRAKE_KMH_S * 2.0  # Faster braking for safety
       speed_kmh = max(target_speed, speed_kmh - emergency_brake_rate * dt)
   
   speed_kmh = clamp(speed_kmh, 0, MAX_KMH)
   # ---------- Smooth lane change ----------
   desired_offset_px = (target_lane - center_lane) * lane_width_bottom
   delta = desired_offset_px - curr_lane_offset_px
   max_step = LANE_CHANGE_SPEED * dt
   if abs(delta) <= max_step:
       curr_lane_offset_px = desired_offset_px
   else:
       curr_lane_offset_px += math.copysign(max_step, delta)
   # ---------- Camera dynamics: yaw & pitch ----------
   lateral_vel_px_s = (curr_lane_offset_px - prev_lane_offset_px) / dt_safe
   yaw_target_px = YAW_FROM_STEER * lateral_vel_px_s
   yaw_alpha = clamp(dt * YAW_SMOOTH_HZ, 0.0, 1.0)
   yaw_px = lerp(yaw_px, yaw_target_px, yaw_alpha)
   dv_kmh = (speed_kmh - prev_speed_kmh)
   accel_ms2 = (dv_kmh * (1000.0 / 3600.0)) / dt_safe
   pitch_target_px = PITCH_FROM_ACCEL * accel_ms2
   pitch_alpha = clamp(dt * PITCH_SMOOTH_HZ, 0.0, 1.0)
   pitch_px = lerp(0.0, pitch_target_px, pitch_alpha)  # transient-only feel
   horizon_y = clamp(HORIZON_BASE + pitch_px, int(H * 0.22), int(H * 0.55))
   prev_lane_offset_px = curr_lane_offset_px
   prev_speed_kmh = speed_kmh
   # ---------- World geometry ----------
   cam_offset_px = curr_lane_offset_px + driver_offset_px
   road_half_bottom = ROAD_BOTTOM_WIDTH / 2
   vp_x = W // 2 + int(yaw_px)
   left_bottom_x  = W // 2 - road_half_bottom - cam_offset_px
   right_bottom_x = W // 2 + road_half_bottom - cam_offset_px
   # ---------- Move lane dashes (screen-space illusion is fine here) ----------
   speed_px_s = speed_kmh * PX_PER_KMH
   track_len = (H - horizon_y) + 2 * DASH_SPACING
   for i in range(len(dash_rows)):
       y = dash_rows[i] + speed_px_s * dt
       if y > H + DASH_SPACING:
           y -= track_len
       dash_rows[i] = y
   # ---------- Car Management ----------
   # Update horn feedback timer
   if horn_feedback_timer > 0:
       horn_feedback_timer -= dt
   
   # Update LICHTHUPE flash timer
   if lichthupe_flash_timer > 0:
       lichthupe_flash_timer -= dt
   
   # Spawn new cars
   car_spawn_timer += dt
   if car_spawn_timer >= 1.0 / CAR_SPAWN_RATE:
       car_spawn_timer = 0.0
       if len(cars) < 12:  # reduced from 20 to allow more space for overtaking
           cars.append(spawn_car())
   # Update & cull
   for car in cars[:]:
       car.update(dt, speed_kmh)
       if car.is_offscreen():
           cars.remove(car)
   # ---------- Draw ----------
   screen.fill(BG)
   
   # LICHTHUPE flash effect - dramatic white flash overlay
   if lichthupe_flash_timer > 0:
       # Create pulsing effect - brighter at the beginning
       flash_intensity = (lichthupe_flash_timer / 0.3) * 255
       flash_intensity = min(255, max(0, flash_intensity))
       
       # Create flash overlay
       flash_surface = pygame.Surface((W, H))
       flash_surface.set_alpha(int(flash_intensity * 0.7))  # 70% opacity at peak
       flash_surface.fill((255, 255, 255))  # White flash
       screen.blit(flash_surface, (0, 0))
       
       # Big LICHTHUPE text
       big_font = pygame.font.SysFont(None, 120, bold=True)
       lichthupe_text = big_font.render("LICHTHUPE", True, (255, 255, 255))
       text_rect = lichthupe_text.get_rect(center=(W // 2, H // 2))
       
       # Add black outline for better visibility
       outline_positions = [(-2, -2), (-2, 2), (2, -2), (2, 2)]
       for dx, dy in outline_positions:
           outline_text = big_font.render("LICHTHUPE", True, (0, 0, 0))
           screen.blit(outline_text, (text_rect.x + dx, text_rect.y + dy))
       
       screen.blit(lichthupe_text, text_rect)
   
   # Road trapezoid
   road_poly = [
       (vp_x - ROAD_TOP_HALF_WIDTH, horizon_y),
       (vp_x + ROAD_TOP_HALF_WIDTH, horizon_y),
       (right_bottom_x, H),
       (left_bottom_x, H),
   ]
   pygame.draw.polygon(screen, ROAD, road_poly)
   # Road edges
   pygame.draw.line(screen, EDGE, (vp_x - ROAD_TOP_HALF_WIDTH, horizon_y), (left_bottom_x, H), 4)
   pygame.draw.line(screen, EDGE, (vp_x + ROAD_TOP_HALF_WIDTH, horizon_y), (right_bottom_x, H), 4)
   # Lane dashed lines
   boundary_fs = [(i / LANES) for i in range(1, LANES)]
   for f in boundary_fs:
       for y in dash_rows:
           if y <= horizon_y + 1:
               continue
           # perspective scale factor using current horizon
           t = clamp((y - horizon_y) / (H - horizon_y), 0.0, 1.0)
           s = max(0.12, t ** 1.25)
           dash_h = max(2, int(DASH_BASE_LEN * s))
           dash_w = max(2, int(DASH_BASE_W * s))
           x = lane_x_at_y(y, f, vp_x, left_bottom_x, right_bottom_x, horizon_y)
           rect = pygame.Rect(0, 0, dash_w, dash_h)
           rect.center = (int(x), int(y))
           pygame.draw.rect(screen, DASH, rect, border_radius=2)
   # Draw cars (far-to-near so nearer cars are on top)
   for car in sorted(cars, key=lambda c: c.z_m, reverse=True):
       rect = car.get_screen_rect(vp_x, left_bottom_x, right_bottom_x, horizon_y)
       if rect:
           # Use car images if available, otherwise fallback to colored rectangles
           if game_assets.get('car_images'):
               draw_car_with_image(screen, car, rect, game_assets['car_images'])
           else:
               pygame.draw.rect(screen, CAR_COLOR, rect)
   # Cockpit/hood - use image if available, otherwise draw polygon
   if game_assets.get('cockpit'):
       hood_height = draw_cockpit(screen, game_assets['cockpit'])
   else:
       hood_height = 115
       hood_poly = [
           (0, H),
           (W, H),
           (int(W * 0.78), H - hood_height),
           (int(W * 0.22), H - hood_height),
       ]
       pygame.draw.polygon(screen, HOOD, hood_poly)
   # HUD
   spd = font.render(f"{int(speed_kmh):3d} km/h", True, HUD)
   lane_text = font.render(f"Lane: {target_lane + 1} / {LANES}", True, HUD)
   car_count = font.render(f"Cars: {len(cars)}", True, HUD)
   
   # Show acceleration mode
   if keys[pygame.K_UP] or keys[pygame.K_w] or manual_accel:
       if manual_accel:
           accel_mode = font.render("SWIPE ACCEL", True, (255, 255, 100))
       else:
           accel_mode = font.render("MANUAL ACCEL", True, (100, 255, 100))
       screen.blit(accel_mode, (20, 100))
   elif keys[pygame.K_DOWN] or keys[pygame.K_s] or manual_brake:
       if manual_brake:
           accel_mode = font.render("SWIPE BRAKE", True, (255, 150, 100))
       else:
           accel_mode = font.render("BRAKING", True, (255, 100, 100))
       screen.blit(accel_mode, (20, 100))
   elif not car_blocking and speed_kmh < MAX_KMH:
       accel_mode = font.render("AUTO ACCEL", True, (100, 200, 255))
       screen.blit(accel_mode, (20, 100))
   
   screen.blit(spd, (20, 16))
   screen.blit(lane_text, (20, 44))
   screen.blit(car_count, (20, 72))
   
   # Show acceleration blocked indicator
   closest_car, closest_distance = get_closest_car_ahead(cars, target_lane)
   if is_car_too_close_ahead(cars, target_lane):
       blocked_text = font.render("BLOCKED - Car ahead too close!", True, (255, 100, 100))
       screen.blit(blocked_text, (20, 128))
   
   # Show distance to car ahead for debugging/awareness
   if closest_car and closest_distance < 100.0:
       distance_text = font.render(f"Car ahead: {closest_distance:.1f}m", True, HUD)
       screen.blit(distance_text, (20, 156))
       if closest_distance < 50.0:
           warning_text = font.render("FOLLOWING TOO CLOSE", True, (255, 200, 100))
           screen.blit(warning_text, (20, 184))
   
   # Show horn feedback
   if horn_feedback_timer > 0:
       feedback_color = (100, 255, 100) if "moved" in horn_feedback_text else (255, 200, 100)
       feedback_text = font.render(horn_feedback_text, True, feedback_color)
       screen.blit(feedback_text, (20, 212))
   
   # Show controls help
   help_text1 = font.render("Keyboard: Arrows/WASD + H/Space | Serial: Swipe L/R/Up/Down + Push", True, (150, 150, 150))
   help_text2 = font.render("H: Horn | Space/Push: LICHTHUPE | SwipeUp: Accel | SwipeDown: Brake", True, (150, 150, 150))
   screen.blit(help_text1, (20, H - 50))
   screen.blit(help_text2, (20, H - 25))
   
   pygame.display.flip()
pygame.quit()
sys.exit()