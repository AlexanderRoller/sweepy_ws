import serial

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port and baud rate if needed
ser.write(b'\x80\x01\x00\x01\x01\x80')  # Example command to RoboClaw
response = ser.read(7)  # Read response
print(response)
ser.close()
