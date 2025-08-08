import enum
import re
import serial
import sys

# Configure your serial port
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"  # Default to COM15 if no argument is provided
BAUD_RATE = 115200  ## You don't need to change this

class Control(enum.Enum):
    SwipeLeft = 1
    SwipeRight = 2
    SwipeUp = 3
    SwipeDown = 4
    Push = 5

def read_and_parse_serial():
    # Open the serial connection
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")

        while True:
            try:
                # Read a line from the serial port
                line = ser.readline().decode("utf-8").strip()

                # Use a regular expression to extract gestures
                match = re.search(r'\.(\w+)', line)
                if match:
                    gesture = match.group(1)
                    if gesture == "SwipeLeft":
                        return Control.SwipeLeft
                    elif gesture == "SwipeRight":
                        return Control.SwipeRight
                    elif gesture == "SwipeUp":
                        return Control.SwipeUp
                    elif gesture == "SwipeDown":
                        return Control.SwipeDown
                    elif gesture == "Push":
                        return Control.Push
                    else:
                        raise ValueError(f"Unknown gesture: {gesture}")
                    print(f"{gesture}")

            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")


if __name__ == "__main__":
    read_and_parse_serial()