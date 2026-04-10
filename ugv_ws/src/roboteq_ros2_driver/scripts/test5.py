import serial
import time

ser = serial.Serial('/dev/ttyAMA10', 115200, timeout=0.01)

def start_true_infinite_crawl():
    try:
        print("--- COMMENCING CONTINUOUS CRAWL ---")
        print("Now sending heartbeats to bypass the Watchdog. Ctrl+C to stop.")
        
        while True:
            # We RESEND the command every loop to keep the Watchdog happy
            ser.write(b"!G 1 -80\r!G 2 80\r")
            
            # Optional: Read encoders to see progress
            ser.write(b"?C 1\r?C 2\r")
            response = ser.read_all().decode().strip()
            if response:
                print(f"Moving... {response.replace('\r', ' ')}")
            
            # Sleep MUST be less than the watchdog timeout (usually 1000ms)
            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\n[STOP] Kill signal received.")
    finally:
        ser.write(b"!G 1 0\r!G 2 0\r")
        ser.close()
        print("--- Safety Stop Engaged ---")

if __name__ == "__main__":
    start_true_infinite_crawl()
