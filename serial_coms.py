import serial
import platform
# import rospy
# from dynamixel_gui.msg import DynamixelPosition
if platform.system() == "Windows":
    arduino = serial.Serial("\\\\.\\COM8")
elif platform.system() == "Linux":
    arduino = serial.Serial("/dev/ttyUSB0")

arduino.baudrate = 115200


def position(value):
    val = (int(input("Enter value:"))).to_bytes(2, byteorder="little")
    ba_dyn_id = bytearray(2)
    ba_com = bytearray()
    ba_val = bytearray(val)

    ba_dyn_id.append(ba_com[0])
    ba_dyn_id.append(ba_val[1])
    ba_dyn_id.append(ba_val[0])
    arduino.write(ba_dyn_id)


while True:

    dyn_id = (int(input("Enter id (2-5):"))).to_bytes(1, byteorder="little")
    com = (int(input("Enter command type (1 torque) (2 Speed) (3 Postion):"))).to_bytes(1, byteorder="little")
    val = (int(input("Enter value(speed 0-1023)(position 0-4095):"))).to_bytes(2, byteorder="little")

    ba_dyn_id = bytearray(dyn_id)
    ba_com = bytearray(com)
    ba_val = bytearray(val)

    ba_dyn_id.append(ba_com[0])
    ba_dyn_id.append(ba_val[1])
    ba_dyn_id.append(ba_val[0])
    arduino.write(ba_dyn_id)
