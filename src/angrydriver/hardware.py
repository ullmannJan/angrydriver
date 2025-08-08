import re
import serial
import sys

# Configure your serial port
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"  # Default to COM15 if no argument is provided
BAUD_RATE = 115200  ## You don't need to change this


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
                        ## Do something for SwipeLeft
                        pass
                    elif gesture == "SwipeRight":
                        ## Do something for SwipeRight
                        pass
                    elif gesture == "SwipeUp":
                        ## Do something for SwipeUp
                        pass
                    elif gesture == "SwipeDown":
                        ## Do something for SwipeDown
                        pass
                    elif gesture == "Push":
                        ## Do something for Push
                        pass
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