import serial
arduino = serial.Serial("\\\\.\\COM8")
arduino.baudrate = 115200

while True:

    dyn_id = (int(input("Enter id:"))).to_bytes(1, byteorder="little")
    speed = (int(input("Enter speed:"))).to_bytes(2, byteorder="little")
    print(dyn_id,speed)
    b_dyn_id = bytearray(dyn_id)
    b_speed = bytearray(speed)
    print(b_dyn_id[0],b_speed[0],b_speed[1])
    b_dyn_id.append(b_speed[1])
    b_dyn_id.append(b_speed[0])
    print(b_dyn_id)
    print(b_dyn_id[0],b_dyn_id[1],b_dyn_id[2])
    arduino.write(b_dyn_id)
