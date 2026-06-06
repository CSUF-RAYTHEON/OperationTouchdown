# test_lora_tx.py
import serial
import time

# Update this to match how the LoRa is wired to your Drone's Pi.
# '/dev/serial0' (GPIO TX/RX pins) or '/dev/ttyUSB0' (USB adapter)
LORA_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200 # Must match your LoRa module's configured baud rate

def test_transmission():
    try:
        # Open the serial connection to the hardware module
        lora = serial.Serial(LORA_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Drone LoRa on {LORA_PORT}")

        # The UGV code uses .strip().upper(), so a newline is safe and often 
        # required by serial modules to trigger the actual transmission.
        command = "STRAIGHT\n" 
        
        print(f"Broadcasting: {command.strip()}")
        lora.write(command.encode('utf-8'))
        
        # Give the module a moment to finish the RF transmission
        time.sleep(0.5) 
        print("Transmission complete.")

    except Exception as e:
        print(f"Hardware Error: {e}")
    finally:
        if 'lora' in locals() and lora.is_open:
            lora.close()

if __name__ == "__main__":
    test_transmission()