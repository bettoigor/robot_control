#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Library for line following control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 8-5-2022
"""

import rospy,  angles, rostopic, geopy.distance, copy, time
import numpy as np
from utils.control_lib import *
from robot_control.msg import Mission
from solarbot.msg import rtk_fix
from geometry_msgs.msg import Pose2D,Twist
from signal import signal, SIGINT
from solix_bringup.msg import MissionStage, MissionStage2, LineHeader, MissionHeader
from std_msgs.msg import UInt8, Bool, Empty, String


class Discovery():

    def __init__(self):

        # Control parameters
        self.rate = 10


        # Thresholds
        self.linear_threshold = 0
        self.angular_threshold = 0
        self.local_mission = 0
        self.max_v = 0
        self.max_omega = 0
        self.gps_offset = 0
        self.edge_size = 0

        # Control gains
        self.gains_line = None
        self.gains_polar = None
        self.control_weight = None

        # Process control variables
        self.mission = Mission()
        self.stage = None
        self.valid_map = None
        self.mission_header = MissionHeader()
        self.curr_pose = Pose2D()
        self.gps_offset = 0
        self.cmd_vel_img = Twist()
        self.cmd_vel_lin = Twist()
        self.abort = False
        self.robot_pose = Pose2D()

        # Control object
        self.discovery_control = LineControl()
        self.in_edge = True


        print("Discovery Control Object successfully created!")

    def set_subscribers(self):
        # Method ok!

        # Creating subscribers
        rospy.Subscriber('rtk',rtk_fix,self.callback_rtk)
        rospy.Subscriber('cmd_vel_img', Twist, self.callback_cmd_vel_img)
        rospy.Subscriber('mission2', MissionStage2, self.callback_mission_2)
        rospy.Subscriber('abort_mission', Bool, self.callback_abort_mission)


    def set_publishers(self):

        # Method ok!

        # Creating publishers
        self.start_map_pub = rospy.Publisher('start_map',Bool, queue_size=10)
        self.start_visual_servoing_pub = rospy.Publisher('start_visual', Bool, queue_size=10)
        self.reset_visual_servoing_pub = rospy.Publisher('reset_visual', Bool, queue_size=10)
        self.save_mission_pub = rospy.Publisher('save_mission', UInt8, queue_size=10)
        self.save_map_pub = rospy.Publisher('save_map',Bool, queue_size=10)
        self.line_header_pub = rospy.Publisher('line_header',LineHeader,queue_size=10)
        self.mission_header_pub = rospy.Publisher('mission_header', MissionHeader, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals
        self.save_line_pub = rospy.Publisher('save_line',Bool, queue_size=10)

    def exiting(self, signal_received, frame):

        # Method ok!

        # Saving the map
        self.save_mission_pub.publish(self.stage)

        # Stopping the robot
        self.cmd_vel_pub.publish(Twist())

        # Handle any cleanup here
        print('SIGINT or CTRL-C detected. Exiting gracefully')


    def callback_mission_2(self, data):

        # Method ok!

        # Saving mission header
        self.mission_header = data.header
        self.stage = self.mission_header.stage

        # Saving the lines of the mission
        self.map_lines = data.lines

        print("\n---\nMission successfully received from externa API!\n")


    def callback_rtk(self, data):

         # Method ok!

        gps_data = data

        # Saving gps data
        lat = gps_data.latitude
        lon = gps_data.longitude
        heading = np.deg2rad(gps_data.heading + self.gps_offset)

        self.curr_pose.x = lat
        self.curr_pose.y = lon
        self.curr_pose.theta = heading


    def callback_cmd_vel_img(self, data):

        # Method ok!

        # Getting velocity command from image controller
        self.cmd_vel_img = data


    def callback_abort_mission(self, data):

        # Method ok!

        # Aborting mission and node
        self.abort = data.data

        # Saving the map
        self.save_mission_pub.publish(self.stage)

        rospy.signal_shutdown("Mission Aborted!")

        print("\n\n #### Mission Aborted! #### \n\n")


    def set_mission(self,points, mission):

        for i, p in enumerate(points):

            d = geopy.distance.geodesic((mission.start_latitude, mission.start_longitude),
                                        (p.latitude, p.longitude)).m

            if d >= self.edge_size:
                mission.stop_latitude = p.latitude
                mission.stop_longitude = p.longitude

                return mission



    def run(self):
        print("Discovery Library running...")

        # Initializing ros node
        rospy.init_node('line_following_node',anonymous=True)

        # Starting node
        self.set_subscribers()
        self.set_publishers()

        # Node rate
        node_rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            # Mission control
            if not self.stage:
                # No mission receive yet
                print(" *** Waiting mission ***", end="\r")

            else:
                # Mission received
                print("*** Mission Received from API. Starting robot control.\n")
                print(f"\n *** Running mission {self.stage} ***")

                # Sending mission header to map creator
                self.mission_header_pub.publish(self.mission_header)
                print(f"\nSent new mission header\n{self.mission_header}\n")
                time.sleep(1)

                # Starting line creation/validation
                for index, line in enumerate(self.map_lines):

                    # Getting the reference points
                    self.mission.start_latitude = line.points[0].latitude
                    self.mission.start_longitude = line.points[0].longitude

                    line_header = line.header
                    line_header.start = True

                    # Sending command to start the point collection
                    self.line_header_pub.publish(line_header)
                    line.points.pop(0)
                    print(f"---\nLine header sent:\n{line_header}")

                    # Setting mission details
                    self.mission = self.set_mission(line.points, self.mission)
                    print(f"Mission:\n{self.mission}")

                    # Setting status as IN_MISSION
                    in_mission = True

                    # Defining the submission type as TO_START
                    to_start = True

                    # Setting the mission in the controller
                    self.discovery_control.set_line(self.mission, local=True)

                    # Setting the robot current pose as origin
                    self.discovery_control.set_origin(copy.deepcopy(self.curr_pose))

                    # Resetting the integral therm
                    self.discovery_control.reset_integral()

                    print(f"Mission Line {index}:\n{self.mission}")

                    # Mission loop
                    while in_mission and not self.abort:
                        # Starting the control signal variable
                        cmd_vel = Twist()

                        if to_start:
                            print(f"*** Mission {self.stage}: Going to first point of the line {index} ***")

                            # Computing the control signal based on robot position
                            cmd_vel, ang_err, rho = self.discovery_control.go_to_pose(self.curr_pose,
                                                                                 destination="start",
                                                                                 line_gains=self.gains_line,
                                                                                 polar_gains=self.gains_polar,
                                                                                 local_coord=self.local_mission)
                            cmd_vel.linear.x = cmd_vel.linear.x if abs(cmd_vel.linear.x) <= self.max_v else np.sign(
                                cmd_vel.linear.x) * self.max_v

                            print(f"*** Mission Status ***\n/"
                                  f"   Distance: {rho}\n/"
                                  f"   Angular Err: {ang_err}\n/"
                                  f"  In edge: {self.in_edge}")

                            # Stop condition
                            if abs(ang_err) <= self.angular_threshold and rho <= self.linear_threshold:
                                print("\nReach the stop condition.")
                                if self.stage == 2 and self.in_edge:
                                    print("\nReach the line start point.\n/"
                                          "Going to crop start point...")
                                    # Continue to the next start point
                                    to_start = True

                                    # Change the location to going outside the edge
                                    self.in_edge = False

                                    # Getting the new reference points
                                    self.mission.start_latitude = self.mission.stop_latitude
                                    self.mission.start_longitude = self.mission.stop_longitude
                                    line.points.pop(0)

                                    for i, p in enumerate(line.points):
                                        d = geopy.distance.geodesic((self.mission.start_latitude, self.mission.start_longitude),
                                                                    (p.latitude, p.longitude)).m
                                        print(f"Start: ({self.mission.start_latitude}, {self.mission.start_longitude})\n"
                                              f"Stop (candidate): ({p.latitude}, {p.longitude})\n"
                                              f"Distance: {d}")
                                        if d >= self.edge_size:
                                            self.mission.stop_latitude = p.latitude
                                            self.mission.stop_longitude = p.longitude
                                            print(f"Second Candidate defined:\n{p}\nDistance: {d}")
                                            break

                                    # Setting the mission in the controller
                                    self.discovery_control.set_line(self.mission, local=True)

                                    # Setting the robot current pose as origin
                                    self.discovery_control.set_origin(copy.deepcopy(self.curr_pose))

                                    # Resetting the integral therm
                                    self.discovery_control.reset_integral()


                                else:

                                    # Stop TO START routine
                                    to_start = False

                                    # Reset the edge flag
                                    self.in_edge = True

                                    # Setting the new reference point to final point
                                    self.mission.stop_latitude = line.points[-1].latitude
                                    self.mission.stop_longitude = line.points[-1].longitude

                                    # Setting the mission in the controller
                                    self.discovery_control.set_line(self.mission, local=True)

                                    # Start the visual_servoing controller
                                    self.start_visual_servoing_pub.publish(True)

                                    # Start saving map
                                    self.start_map_pub.publish(True)

                                    # Set all robot velocities to zero
                                    cmd_vel = Twist()

                                    print("The robot reached the start point.")


                        else:
                            print(f"*** Mission {self.stage}: Going to last point of the line {index} ***")

                            # Getting the signal from line follower controller
                            cmd_vel_lin, ang_err, rho = self.discovery_control.go_to_pose(self.curr_pose,
                                                                                     destination="stop",
                                                                                     line_gains=self.gains_line,
                                                                                     polar_gains=self.gains_polar,
                                                                                     local_coord=self.local_mission)

                            cmd_vel_lin.linear.x = cmd_vel_lin.linear.x if abs(cmd_vel_lin.linear.x) <= self.max_v else np.sign(
                                cmd_vel_lin.linear.x) * self.max_v

                            print(f"Mission State:\n   Distance: {rho}\n   Angular Err: {ang_err}")

                            # Creating the velocity variables
                            # Using both visual servoing (mostly) and GPS inside the talhao
                            cmd_vel.linear.x = (self.control_weight[1] * cmd_vel_lin.linear.x + \
                                                (1 - self.control_weight[1]) * self.cmd_vel_img.linear.x) * (abs(np.tanh(rho)))
                            cmd_vel.angular.z = self.control_weight[1] * cmd_vel_lin.angular.z + \
                                                (1 - self.control_weight[1]) * self.cmd_vel_img.angular.z

                            # Using only GPS while outside the talhao (°|°)
                            if rho < self.edge_size and self.stage == 2:
                                print("Inside the edge (bordadura) - Using only GPS")
                                cmd_vel.linear.x = (self.control_weight[0] * cmd_vel_lin.linear.x) #* (abs(np.tanh(rho)))
                                cmd_vel.angular.z = self.control_weight[0] * cmd_vel_lin.angular.z

                                # Start the visual_servoing controller
                                # start_visual_servoing_pub.publish(False)

                            if abs(ang_err) <= self.angular_threshold and rho <= self.linear_threshold:
                                # Stop robot
                                cmd_vel = Twist()

                                # Stop visual servoing control signal
                                self.start_visual_servoing_pub.publish(False)

                                # Saving the map
                                self.save_mission_pub.publish(self.stage)

                                # Ending mission
                                in_mission = False

                                print("The robot reached the last point of the line.\nGoing to the next line.\n\n")

                        # Velocity threshold
                        cmd_vel.linear.x = cmd_vel.linear.x if abs(cmd_vel.linear.x) <= self.max_v else np.sign(
                            cmd_vel.linear.x) * self.max_v
                        # cmd_vel.linear.x = np.sign(cmd_vel.linear.x) * max_v

                        cmd_vel.angular.z = cmd_vel.angular.z if abs(cmd_vel.angular.z) <= self.max_omega else np.sign(
                            cmd_vel.angular.z) * self.max_omega

                        # Publishing the velocity command
                        self.cmd_vel_pub.publish(cmd_vel)

                        #signal(SIGINT, exiting)
                        node_rate.sleep()

                    # Sending command to stop the data collection
                    self.save_line_pub.publish(True)
                    print(f"\nLine saved:\n{line_header}")

                    time.sleep(0.5)


                print("All lines validates. Saving and sending map...")
                # Saving created/validated map
                self.save_map_pub.publish(True)


                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)

                # Assuming the mission is accomplished
                print("Mission Successfully completed!")
                self.stage = 0



            node_rate.sleep()
