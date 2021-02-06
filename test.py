import can
from math import pi
import struct
import time

wheel_radius = 0.1    # lun ban jing
wheel_distance = 0.4    # lun jian ju
id = 601    # driver id
data = [0x23,0x00,0x20,0x01,0x64,0x00,0x00,0x00]     # 601#2300200164000000
stop = [0x2C,0x0E,0x20,0x01,0x00,0x00,0x00,0x00]

bus = can.Bus(interface='socketcan',
              channel='can0',
              receive_own_messages=True)

def speed_set(RobotV,YawRate):
    """if YawRate == 0:
        leftV = RobotV
        rightV = RobotV
    elif RobotV == 0:
        rightV = YawRate / 2.0
        leftV = -rightV
    else:
        leftV =  RobotV - YawRate / 2.0
        rightV = RobotV + YawRate / 2.0
    print ("leftV=", leftV, ", rightV=", rightV)
    LeftWheelV  = (leftV * 60 / 2 / pi / radius)
    RightWheelV = (rightV * 60 / 2 / pi / radius)
    """
    LeftWheelSpeed, RightWheelSpeed = int((RobotV - YawRate*wheel_distance/2) * 30 / (pi * wheel_radius)), int((RobotV + YawRate*wheel_distance/2) * 30 / (pi * wheel_radius))
    print ("LeftWheelSpeed=", LeftWheelSpeed, ", RightWheelSpeed=", RightWheelSpeed)
    speed_motorL_low = struct.pack("<h",LeftWheelSpeed)[-2]
    speed_motorL_high = struct.pack("<h",LeftWheelSpeed)[-1]

    speed_motorR_low = struct.pack("<h",RightWheelSpeed)[-2]
    speed_motorR_high = struct.pack("<h",RightWheelSpeed)[-1]
    return speed_motorL_low, speed_motorL_high, speed_motorR_low, speed_motorR_high


def callback(RobotV, YawRate):
    # RobotV = cmd_input.linear.x
    # YawRate = cmd_input.angular.z
    if RobotV==0 and YawRate==0:
        time.sleep(0.2)
    else:
        speed_motorL_low, speed_motorL_high, speed_motorR_low, speed_motorR_high = speed_set(RobotV,YawRate)
        # left wheel
        data[-4], data[-3] = speed_motorL_low, speed_motorL_high
        message = can.Message(arbitration_id=id, is_extended_id=True, data=data)
        bus.send(message, timeout=0.2)

        time.sleep(0.01)
        # right wheel
        data[-4], data[-3] = speed_motorR_low, speed_motorR_high
        message = can.Message(arbitration_id=id, is_extended_id=True,data=data)
        bus.send(message, timeout=0.2)

	# ser.write(speed_motorL)
	# time.sleep(0.01)
	# ser.write(speed_motorR)


if __name__ == '__main__':
    RobotV, YawRate = 0, pi
    callback(RobotV, YawRate)