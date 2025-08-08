# autobahn_bmw_driver_view.py

import sys
import math
import pygame

from angrydriver.hardware import read_and_parse_serial

 

# ---------- Config ----------

W, H = 900, 600

FPS = 60

ROAD_BOTTOM_WIDTH = 1600

ROAD_TOP_HALF_WIDTH = 8            # tiny near-horizon width

HORIZON_BASE = int(H * 0.50)       # ~eye level for driver view (moved closer)

LANES = 5

MAX_KMH = 300

ACCEL_KMH_S = 10

BRAKE_KMH_S = 161

DRAG_KMH_S = 15

PX_PER_KMH = 2.0                   # reduced from 3.2 to slow down visual speed

LANE_CHANGE_SPEED = 650

 

DASH_SPACING = 60

DASH_BASE_LEN = 28

DASH_BASE_W = 6

 

# Car spawning config

CAR_SPAWN_RATE = 0.6               # cars per second (increased slightly)

CAR_BASE_WIDTH = 45                # car width at bottom of screen

CAR_BASE_HEIGHT = 90               # car height at bottom of screen

CAR_MIN_SPEED_KMH = 80             # minimum car speed

CAR_MAX_SPEED_KMH = 160            # maximum car speed (more reasonable range)

CAR_COLOR = (0, 0, 0)              # black for now

 

# Driver-view tuning

DRIVER_OFFSET_IN_LANE = -0.18      # fraction of lane width; negative = sit left

YAW_FROM_STEER = 0.015             # pixels of VP shift per px/s lateral velocity

PITCH_FROM_ACCEL = -0.9            # pixels horizon moves per m/s^2 accel (negative = horizon drops on accel)

YAW_SMOOTH_HZ = 8.0                # higher = snappier

PITCH_SMOOTH_HZ = 6.0              # higher = snappier

 

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

 

# Perspective lerp anchored at the (dynamic) horizon.

def lane_x_at_y(y, f, vp_x, left_bottom_x, right_bottom_x, horizon_y):

    t = clamp((y - horizon_y) / (H - horizon_y), 0.0, 1.0)  # 0 at horizon, 1 at bottom

    x_bottom = left_bottom_x + (right_bottom_x - left_bottom_x) * f

    return vp_x + (x_bottom - vp_x) * t

 

# ---------- Car System ----------

class Car:

    def __init__(self, lane, y_pos, speed_kmh):

        self.lane = lane                # which lane (0 to LANES-1)

        self.y_pos = y_pos             # world Y position

        self.speed_kmh = speed_kmh     # car's speed in km/h

        self.world_z = 0               # distance from player (for future use)

   

    def update(self, dt, player_speed_kmh, speed_px_s):

        # Calculate relative speed difference (positive means car is faster than player)

        speed_diff_kmh = self.speed_kmh - player_speed_kmh

        speed_diff_px_s = speed_diff_kmh * PX_PER_KMH

       

        # Move car based on speed difference, but cap the maximum relative movement

        # This prevents cars from flying off too quickly when player accelerates hard

        max_relative_speed = 150.0 * PX_PER_KMH  # max 150 km/h relative difference

        capped_speed_diff = max(-max_relative_speed, min(max_relative_speed, speed_diff_px_s))

       

        self.y_pos += capped_speed_diff * dt

   

    def is_offscreen(self, horizon_y):

        return self.y_pos < horizon_y - 50 or self.y_pos > H + 100

   

    def get_screen_rect(self, vp_x, left_bottom_x, right_bottom_x, horizon_y):

        if self.y_pos <= horizon_y:

            return None

       

        # Calculate perspective scale

        t = clamp((self.y_pos - horizon_y) / (H - horizon_y), 0.0, 1.0)

        scale = max(0.1, t ** 1.2)

       

        # Get lane center position

        lane_center_f = (self.lane + 0.5) / LANES

        car_x = lane_x_at_y(self.y_pos, lane_center_f, vp_x, left_bottom_x, right_bottom_x, horizon_y)

       

        # Scale car dimensions

        car_width = max(4, int(CAR_BASE_WIDTH * scale))

        car_height = max(6, int(CAR_BASE_HEIGHT * scale))

       

        return pygame.Rect(

            int(car_x - car_width // 2),

            int(self.y_pos - car_height // 2),

            car_width,

            car_height

        )

 

def spawn_car():

    """Spawn a new car at a random lane near the horizon"""

    import random

    lane = random.randint(0, LANES - 1)

    speed = random.uniform(CAR_MIN_SPEED_KMH, CAR_MAX_SPEED_KMH)

    y_pos = HORIZON_BASE + random.uniform(-60, -20)  # spawn well behind horizon

    return Car(lane, y_pos, speed)

 

def make_dash_rows(spacing, start_y):

    ys = []

    y = start_y - spacing

    while y < H + spacing:

        ys.append(y)

        y += spacing

    return ys

 

# ---------- Init ----------

def run():

    pygame.init()

    screen = pygame.display.set_mode((W, H))

    pygame.display.set_caption("BMW Autobahn - Driver View")

    clock = pygame.time.Clock()

    font = pygame.font.SysFont(None, 28)

    


    center_lane = (LANES - 1) / 2.0

    target_lane = int(round(center_lane))

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

    car_spawn_timer = 0.0

    running = True

    while running:

        dt = clock.tick(FPS) / 1000.0

        dt_safe = max(dt, 1e-4)

        control = read_and_parse_serial() 

        match control:
            case 1:
                target_lane = max(0, target_lane - 1)
            case 2:
                target_lane = min(LANES - 1, target_lane + 1)


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

        # Lateral velocity → look (yaw) slightly into the lane change

        lateral_vel_px_s = (curr_lane_offset_px - prev_lane_offset_px) / dt_safe

        yaw_target_px = YAW_FROM_STEER * lateral_vel_px_s

        yaw_alpha = clamp(dt * YAW_SMOOTH_HZ, 0.0, 1.0)

        yaw_px = lerp(yaw_px, yaw_target_px, yaw_alpha)

    

        # Longitudinal accel → suspension pitch (squat/dive)

        dv_kmh = (speed_kmh - prev_speed_kmh)

        accel_ms2 = (dv_kmh * (1000.0 / 3600.0)) / dt_safe

        pitch_target_px = PITCH_FROM_ACCEL * accel_ms2

        pitch_alpha = clamp(dt * PITCH_SMOOTH_HZ, 0.0, 1.0)

        pitch_px = lerp(0.0, pitch_target_px, pitch_alpha)  # transient-only feel

        horizon_y = clamp(HORIZON_BASE + pitch_px, int(H * 0.22), int(H * 0.55))

    

        prev_lane_offset_px = curr_lane_offset_px

        prev_speed_kmh = speed_kmh

    

        # ---------- World geometry ----------

        # Camera sits left-of-center inside the lane

        cam_offset_px = curr_lane_offset_px + driver_offset_px

        road_half_bottom = ROAD_BOTTOM_WIDTH / 2

        vp_x = W // 2 + int(yaw_px)  # slight look left/right on steer

    

        left_bottom_x  = W // 2 - road_half_bottom - cam_offset_px

        right_bottom_x = W // 2 + road_half_bottom - cam_offset_px

    

        # ---------- Move lane dashes ----------

        speed_px_s = speed_kmh * PX_PER_KMH

        track_len = (H - horizon_y) + 2 * DASH_SPACING

        for i in range(len(dash_rows)):

            y = dash_rows[i] + speed_px_s * dt

            if y > H + DASH_SPACING:

                y -= track_len

            dash_rows[i] = y

    

        # ---------- Car Management ----------

        # Spawn new cars

        car_spawn_timer += dt

        if car_spawn_timer >= 1.0 / CAR_SPAWN_RATE:

            car_spawn_timer = 0.0

            if len(cars) < 20:  # limit max cars to prevent lag

                cars.append(spawn_car())

    

        # Update existing cars

        for car in cars[:]:  # copy list to safely remove during iteration

            car.update(dt, speed_kmh, speed_px_s)

            if car.is_offscreen(horizon_y):

                cars.remove(car)

    

        # ---------- Draw ----------

        screen.fill(BG)

    

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

    

        # Lane dashed lines (boundaries at i/LANES)

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

    

        # Draw cars

        for car in cars:

            car_rect = car.get_screen_rect(vp_x, left_bottom_x, right_bottom_x, horizon_y)

            if car_rect:

                pygame.draw.rect(screen, CAR_COLOR, car_rect)

    

        # Cockpit/hood (bias right so it feels like you're seated left)

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

        screen.blit(spd, (20, 16))

        screen.blit(lane_text, (20, 44))

        screen.blit(car_count, (20, 72))

    

        pygame.display.flip()

    

    pygame.quit()

    sys.exit()
