import serial
arduino = serial.Serial("\\\\.\\COM8")
arduino.baudrate = 115200

while True:

    dyn_id = (int(input("Enter id:"))).to_bytes(1, byteorder="little")
    com = (int(input("Enter command type (1 torque) (2 Speed) (3 Postion):"))).to_bytes(1, byteorder="little")
    val = (int(input("Enter value:"))).to_bytes(2, byteorder="little")

    ba_dyn_id = bytearray(dyn_id)
    ba_com = bytearray(com)
    ba_val = bytearray(val)
    
    ba_dyn_id.append(ba_com[0])
    ba_dyn_id.append(ba_val[1])
    ba_dyn_id.append(ba_val[0])
    arduino.write(ba_dyn_id)
