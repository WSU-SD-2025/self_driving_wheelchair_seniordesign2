import serial
import datetime
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_FILE = f"/home/sejunmoon/wheelchair_matlab/data/voltage_to_velocity_test.csv"

def is_valid_csv(line: str) -> bool:
    line = line.strip()

    if not line:
        return False
    
    if line.startswith("#"):
        return False
    
    if line == "time_ms,phase,step_idx,x_v,y_v,left_count,right_count":
        return True
    
    parts = line.split(",")
    if len(parts) != 7:
        return False
    
    return True

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    print(f"Saving to {OUTPUT_FILE}")
    print("Press Ctrl+C to stop.\n")

    header_written = False

    try:
        with open(OUTPUT_FILE, "w") as f:
            while True:
                raw = ser.readline()

                if not raw:
                    continue

                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except:
                    continue

                if not line:
                    continue

                print(line)

                if not is_valid_csv(line):
                    continue

                if line.startswith("time_ms,phase"):
                    if not header_written:
                        f.write(line + "\n")
                        header_written = True
                        print("[HEADER SAVED]")
                    continue

                if not header_written:
                    f.write("time_ms,phase,step_idx,x_v,y_v,left_count,right_count\n")
                    header_written = True

                f.write(line + "\n")
                f.flush()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()