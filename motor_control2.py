# Last Updated 21/9/2023
# Updated by Antonio Fernando Christophorus

from turtle import speed
import numpy as np
from Ax12 import Ax12
import time

LENS_RADIUS = 129.20 #from edmunds
DIST_TO_FL = 112.5
DIST_TO_LENS = 100
AIR_REFRACTIVE = 1
LENS_REFRACTIVE = 1.5

DEG_TO_RAD = np.pi/180
RAD_TO_DEG = 180/np.pi

MAX_X_ANGLE = 8  * DEG_TO_RAD
MAX_Y_ANGLE = 11 * DEG_TO_RAD

# NEED ORIENTATION ON LENS (i.e. flat or round on top)
def ray_tracing(angle, position):
    wanted_position = np.array([position, angle])
    
    # Travel from focal plane to lens
    if angle == 0:
        to_FL = DIST_TO_FL
    else:     
        to_FL = DIST_TO_FL/np.cos(angle)

    after_lens = np.array(([1,to_FL],[0,1]))

    # Travel through lens
    lens_flat = np.array(([1,0],[0,AIR_REFRACTIVE/LENS_REFRACTIVE]))
    lens_2 = np.eye(2)
    lens_round = np.array(([1,0],[-1/LENS_RADIUS * (1-LENS_REFRACTIVE/AIR_REFRACTIVE),LENS_REFRACTIVE/AIR_REFRACTIVE]))

    lens_matrix = lens_round @ lens_2 @ lens_flat

    position_before_lens = np.linalg.inv(lens_matrix) @ np.linalg.inv(after_lens) @ wanted_position

    # Travel through air from mirror to lens
    if position_before_lens[1] == 0:
        to_lens = DIST_TO_FL
    else:     
        to_lens = DIST_TO_FL/np.cos(position_before_lens[1])

    after_lens = np.array(([1,to_lens],[0,1]))


    position = np.linalg.inv(after_lens) @ position_before_lens

    angle_at_mirror = position[1]

    return angle_at_mirror

def angle_to_FL(wanted_position):
    return wanted_position[0]*MAX_X_ANGLE, wanted_position[1]*MAX_Y_ANGLE

def find_mirror_angle(angle_at_mirror):
    normal_angle  = (np.pi/2 - 30*DEG_TO_RAD - angle_at_mirror)/2

    return normal_angle + np.pi/2

def from_wanted_to_angle(wanted_position):
    x_angle, y_angle = angle_to_FL(wanted_position)
    
    angle_wanted_x = ray_tracing(x_angle, wanted_position[0])
    angle_wanted_y = ray_tracing(y_angle, wanted_position[1])

    mirror_x = find_mirror_angle(angle_wanted_x) - 120 * DEG_TO_RAD
    mirror_y = find_mirror_angle(angle_wanted_y) - 120 * DEG_TO_RAD
    
    return mirror_x, mirror_y


if __name__ == "__main__": 
    wanted_position = np.array([0,0])

    mirror_x = from_wanted_to_angle(wanted_position)[0]
    mirror_y = from_wanted_to_angle(wanted_position)[1]

    # print(mirror_x * RAD_TO_DEG)
    # print(mirror_y * RAD_TO_DEG)


#  ---------------------------   ---------------------------    ---------------------------    ---------------------------    ---------------------------
""" Integration with Dynamixel Ax-12 """
#  ---------------------------   ---------------------------    ---------------------------    ---------------------------    ---------------------------


# Define ports used by running commands below on terminal/command prompt
# use 'COM3' for windows (open device manager(Advanced Menu) -> View Hidden Device -> Ports(Com & LPT))  
# use '/dev/ttyUSB0' for Linux
# use ls /dev/tty.usb* for Mac

Ax12.DEVICENAME = '/dev/tty.usbserial-FT7WBAZG' # Define port names here that is connected to the motor 

Ax12.BAUDRATE = 1_000_000 

# Sets baudrate and opens com port
Ax12.connect()

# Create AX12 instance with specified ID on motor
motor_id_1 = 10
motor_id_2 = 11
dxl_1 = Ax12(motor_id_1)  
dxl_2 = Ax12(motor_id_2)  

def main(dxl_1,dxl_2,positions):
    """"

    Motor Control
    All angles should be minus 30 degrees since position 0 is offset by 30 degrees
    e.g: shaft position = 90 degrees = 90 - 30 = 60 degrees = 60/0.29 = 206.897 = 206 in ax-12 unit
    our 0 degrees refer to 60 degrees as shown in the inital design images
    Range of Speed: 0 ~ 255 
    Range of Position(Joint Mode): 0 ~ 300 degrees
    Range of values to define position of motor: 0(0x000) ~ 1023(0x3ff)
    Unit: 1(0x1) = 0.29 degrees
    e.g: 
    goal position = 250 degrees
                  = 250 / 0.29
                  = 862.07
    
    Basic Function:
    Set the moving speed to goal position: ax12_object.set_moving_speed(0~255)
    Write goal position: ax12_object.set_goal_position(0~1023)
    Read current speed: ax12_object.get_present_speed()
    Disconnect with motor: set_torque_enable(0) and Ax12.disconnect()

    """

    
    """

    Threshold is used to account for discrepancies between the desired motor position
    and the actual motor position. Discrepancies may exceed the threshold due to rounding,
    as the motor only accepts integer values.
    
    """

    # position_motor_1 = int((positions[1] * RAD_TO_DEG)/ 0.29) + 406
    # position_motor_2 = int((positions[0] * RAD_TO_DEG)/ 0.29) + 842
    # dxl_1.set_moving_speed(50)
    # dxl_1.set_goal_position(position_motor_1)
    # dxl_2.set_moving_speed(50)
    # dxl_2.set_goal_position(position_motor_2)
    # # Sleep command is used to give time for motor to get to position before running the next line
    # time.sleep(0.5)


    for i in range(len(positions)):
        position_motor_1 = int((positions[i][1] * RAD_TO_DEG)/ 0.29) + 406
        position_motor_2 = int((positions[i][0] * RAD_TO_DEG)/ 0.29) + 842
        dxl_1.set_moving_speed(70)
        dxl_1.set_goal_position(position_motor_1)
        dxl_2.set_moving_speed(70)
        dxl_2.set_goal_position(position_motor_2)
        # Sleep command is used to give time for motor to get to position before running the next line
        time.sleep(0.3)

# pass in AX12 object
x_point_1 = np.linspace(-0.7,0.7,20)
y_point_1 = [0.7,-0.7]

locations = []
x_loc = []
y_loc = []
for i in range(len(x_point_1)):
    idx = np.mod(i,2)
    # x_loc.append(from_wanted_to_angle((x_point_1[i],y_point_1[idx]))[0])
    # y_loc.append(from_wanted_to_angle((x_point_1[i],y_point_1)[idx])[1])
    locations.append([from_wanted_to_angle((x_point_1[i],y_point_1[idx]))[0],from_wanted_to_angle((x_point_1[i],y_point_1[idx]))[1]])






# print(locations)
# for i in range(len(x_point_1)):
#     x_loc = from_wanted_to_angle((x_point_1[-i],-y_point_1[-i]))[0]
#     y_loc = from_wanted_to_angle((x_point_1[-i],-y_point_1[-i]))[1]
#     locations.append([x_loc,y_loc])
locations = [[from_wanted_to_angle((-0.7,0.7))[0],from_wanted_to_angle((-0.7,0.7))[1]], [from_wanted_to_angle((-0.7,-0.7))[0],from_wanted_to_angle((-0.7,-0.7))[1]], [from_wanted_to_angle((0.7,-0.7))[0],from_wanted_to_angle((0.7,-0.7))[1]]]
main(dxl_1, dxl_2, locations)

# x_loc = from_wanted_to_angle([1,0])[0]
# y_loc = from_wanted_to_angle([1,0])[1]
# main(dxl_1, dxl_2, [x_loc,y_loc])

# Disconnect
# dxl_1.set_torque_enable(0)
# dxl_2.set_torque_enable(0)
Ax12.disconnect()


