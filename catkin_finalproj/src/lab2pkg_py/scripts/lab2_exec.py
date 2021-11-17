#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
Q11 = [169.35*pi/180.0, -61.20*pi/180.0, 104.44*pi/180.0, -133.88*pi/180.0, -88.46*pi/180.0, 1.83*pi/180.0]
Q12 = [169.35*pi/180.0, -55.74*pi/180.0, 106.26*pi/180.0, -141.16*pi/180.0, -88.46*pi/180.0, 1.84*pi/180.0]
Q13 = [169.35*pi/180.0, -49.24*pi/180.0, 107.02*pi/180.0, -148.38*pi/180.0, -88.46*pi/180.0, 1.85*pi/180.0]
# high position for moving blocks
Q1h = [169.35*pi/180.0, -69.01*pi/180.0, 98.60*pi/180.0, -120.23*pi/180.0, -88.46*pi/180.0, 1.81*pi/180.0]

# Hanoi tower location 2
Q21 = [179.82*pi/180.0, -58.59*pi/180.0, 99.98*pi/180.0, -132.27*pi/180.0, -88.60*pi/180.0, 12.30*pi/180.0]
Q22 = [179.82*pi/180.0, -53.26*pi/180.0, 101.79*pi/180.0, -139.41*pi/180.0, -88.60*pi/180.0, 12.31*pi/180.0]
Q23 = [179.82*pi/180.0, -47.06*pi/180.0, 102.48*pi/180.0, -146.32*pi/180.0, -88.60*pi/180.0, 12.32*pi/180.0]
# high position for moving blocks
Q2h = [179.80*pi/180.0, -65.46*pi/180.0, 94.87*pi/180.0, -120.30*pi/180.0, -88.60*pi/180.0, 12.28*pi/180.0]

# Hanoi tower location 3
Q31 = [188.92*pi/180.0, -54.90*pi/180.0, 93.34*pi/180.0, -129.52*pi/180.0, -88.76*pi/180.0, 21.41*pi/180.0]
Q32 = [188.93*pi/180.0, -49.85*pi/180.0, 95.14*pi/180.0, -136.35*pi/180.0, -88.75*pi/180.0, 21.42*pi/180.0]
Q33 = [188.90*pi/180.0, -43.80*pi/180.0, 94.79*pi/180.0, -142.06*pi/180.0, -88.75*pi/180.0, 21.40*pi/180.0]
# high position for moving blocks 
Q3h = [188.92*pi/180.0, -61.56*pi/180.0, 87.93*pi/180.0, -117.45*pi/180.0, -88.75*pi/180.0, 21.39*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_bool = True
suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13, Q1h], \
      [Q21, Q22, Q23, Q2h], \
      [Q31, Q32, Q33, Q3h] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global suction_bool

    if(msg.AIN0 > 1.9):
        suction_bool = True
    elif(msg.AIN0 < 1.9):
        suction_bool = False



############### Your Code End Here ###############
# def TowerCall(n, source, destination, )

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    # New Code

    # End of New Code

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, end_loc, end_height):
    global Q
    start = Q[start_loc][start_height]
    end = Q[end_loc][end_height]
    vel = 4.0
    acc = 4.0
    # move arm to starting position
    move_arm(pub_cmd, loop_rate, start, vel, acc)
    # grab the block
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(.5)
    if(suction_bool == False):
        move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        print("No Block Detected")
        # kill robot
        gripper(pub_cmd,loop_rate,suction_off)
        return -1
    time.sleep(.5)
    # move to the top position above start 
    move_arm(pub_cmd, loop_rate, Q[start_loc][3], vel, acc)
    # move to the top position above end
    move_arm(pub_cmd, loop_rate, Q[end_loc][3], vel, acc)
    # put the block in the new ending positon 
    move_arm(pub_cmd, loop_rate, end, vel, acc)
    # shut the gripper off
    gripper(pub_cmd,loop_rate,suction_off)
    time.sleep(.5)
    # move arm back to high, middle position 
    move_arm(pub_cmd, loop_rate, Q[1][3], vel, acc)
    time.sleep(.5)
    error = 0
    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    gripper_state = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    tower_start = input("Enter your starting tower (0 to quit): ")
    tower_end = input("Enter your ending tower (0 to quit): ")
    while(tower_start < 0 or tower_start > 3 or tower_end <0 or tower_end > 3 or tower_start == tower_end):
        tower_start = input("Enter your starting tower (0 to quit): ")
        tower_end = input("Enter your ending tower (0 to quit): ")
        
    input_done  = 0
    loop_count = 0
    tower_other = 0

    # 1st
    # 2nd
    # 3rd
    # Pole 1    Pole 2      Pole 3

    # 1st -> End 
    # 2nd -> Other (not start or end )
    # 1st -> Other
    # 3rd -> End
    # 1st -> start 
    # 2nd -> End 
    # 1st -> End

    if((tower_start == 1 and tower_end == 2) or (tower_start == 2 and tower_end == 1)):
        tower_other = 3

    elif((tower_start == 1 and tower_end == 3) or (tower_start == 3 and tower_end == 1)):
        tower_other = 2

    elif((tower_start == 2 and tower_end == 3) or (tower_start == 3 and tower_end == 2)):
        tower_other = 1
    tower_start = tower_start - 1
    tower_end = tower_end - 1
    tower_other = tower_other - 1
    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    # TODO: modify the code so that UR3 can move tower accordingly from user input

    if(move_block(pub_command, loop_rate, tower_start, 0, tower_end,   2) == -1):
        return # 1st -> End 
    if(move_block(pub_command, loop_rate, tower_start, 1, tower_other, 2) == -1): # 2nd -> Other (not start or end )
        return
    if(move_block(pub_command, loop_rate, tower_end,   2, tower_other, 1) == -1):# 1st -> Other
        return
    if(move_block(pub_command, loop_rate, tower_start, 2, tower_end,   2) == -1):# 3rd -> End
        return
    if(move_block(pub_command, loop_rate, tower_other, 1, tower_start, 2) == -1):# 1st -> start 
        return
    if(move_block(pub_command, loop_rate, tower_other, 2, tower_end,   1) == -1): # 2nd -> End 
        return
    if(move_block(pub_command, loop_rate, tower_start, 2, tower_end,   0) == -1):# 1st -> End
        return
if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

# while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)


