import serial
arduino = serial.Serial("/dev/ttyUSB0")
arduino.baudrate = 115200
arduino.write(b"1,4200")
