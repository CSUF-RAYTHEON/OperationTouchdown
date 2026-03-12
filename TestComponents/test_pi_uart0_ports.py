# This code tests the UART RX & TX Pins on the Raspberry Pi using serial0(UART)
# Connect the RX of the Pi to the TX of the Pi, essentially you are sending a message to itself
# using the TX pin of the Pi and then receiving that message using the RX pin of the Pi
# Using one cable connect the RX to the TX of the Raspberry Pi. 

import serial

ser = serial.Serial('/dev/serial0',115200,timeout=1)

while True:
    data = input("Type Something: ")
    ser.write(data.encode())
    received = ser.read(len(data)).decode()
    print("Received: ", received)