#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for robot control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 7-7-2022

"""
import rospy, time, angles,copy, math, sys, os, rostopic
from utils.control_lib import *
import numpy as np
from signal import signal, SIGINT
from robot_control.msg import Speed, MotorStatus

def callback_speed(data):

    global speed_data
    
    speed_data = data


def load_control():

    """
        Loading the control object.
    """

    # communication paramters
    PORT = rospy.get_param("control/communication/port")
    BAUD = rospy.get_param("control/communication/baud_rate")
    TIMEOUT = rospy.get_param("control/communication/timeout")
    
    COMMUNICATION = [PORT,BAUD,TIMEOUT]

    control = RobotControl(communication=COMMUNICATION)

    return control


def exiting(signal_received, frame):

    # Publishers
    global control


    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    control.set_speed(0,0)
    exit(0)


def main():

    global speed_data

    # Starting ros parameters
    rospy.init_node("speed_control_node", anonymous=True)

    # Subscribed nodes
    rospy.Subscriber('speed',Speed,callback_speed)
    h = rostopic.ROSTopicHz(-1)
    rospy.Subscriber('speed', Speed, h.callback_hz, callback_args='speed')
   
    # Publisher nodes
    motor_status = rospy.Publisher('motor_status',MotorStatus,queue_size=10)

    # Creating the control object
    control = load_control()

    RATE = rospy.get_param("control/control/rate")
    rate = rospy.Rate(RATE)


    while not rospy.is_shutdown():

        pub_hz = h.get_hz('speed')

        if pub_hz is not None:
            print("\n***\nSpeed values successfully received!")

            # Filling the speed message
            phi_l = speed_data.left_speed
            phi_r = speed_data.right_speed


        else:
            print("\n***\nNo velocity command received.")
            
            phi_l, phi_r = 0,0


        # Sending the individual speed velocities
        motor_status_data = control.set_speed(phi_l,phi_r)
        motor_status.publish(motor_status_data)


        print(f"\nReceived command: \
              \n Left: {speed_data.left_speed}\
              \n Right: {speed_data.right_speed} \
              \nSending data:\
              \n Left: {phi_l}\n Right: {phi_r}\
              \nMotor Status:\n{motor_status_data}")

        signal(SIGINT, exiting)
        rate.sleep()


#####  Main #####
speed_data = Speed()

print("Starting Low Level Control Node...")

if __name__ == '__main__':

    main()

### End Main