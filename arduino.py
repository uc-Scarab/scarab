import serial
arduino = serial.Serial("/dev/ttyUSB0")
arduino.baudrate = 115200

while True:

    dyn_id = (int(input("Enter id:")))
    speed = (int(input("Enter speed:"))).to_bytes(2, byteorder="little")

    message = bytearray(speed)
    arduino.write(message)
