#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for line following control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 8-5-2022
"""

import rospy,  angles, rostopic, geopy.distance
from utils.control_lib import *
from robot_control.msg import Mission
from solarbot.msg import rtk_fix
from geometry_msgs.msg import Pose2D,Twist
from signal import signal, SIGINT
from solix_bringup.msg import MissionStage
from std_msgs.msg import UInt8, Bool, Empty, String


def callback_mission(data):

    global mission
    global stage
    global validate_map

    stage = data.stage
    validate_map = data.lines

    print("Mission received from API.")

def callback_rtk(data):

    global curr_pose
    global gps_offset

    gps_data = data

    lat = gps_data.latitude
    lon = gps_data.longitude
    heading = np.deg2rad(gps_data.heading+gps_offset)

    curr_pose.x = lat
    curr_pose.y = lon
    curr_pose.theta = heading


def callback_cmd_vel_img(data):
    
    global cmd_vel_img

    cmd_vel_img = data


def callback_abort_mission(data):

    global abort

    abort = data.data


def exiting(signal_received, frame):

    # Publishers
    global cmd_vel_pub
    global stage
    global save_mission_pub


    # Saving the map
    save_mission_pub.publish(stage)

    cmd_vel_pub.publish(Twist())

    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')


def main():
    
    global curr_pose
    global mission
    global cmd_vel_pub
    global discovery_control
    global robot_pose
    global cmd_vel_img
    global stage
    global abort
    global gps_offset
    global save_mission_pub

    print("Discovery node Version 2.0")

    # Initializing ros node
    rospy.init_node('line_following_node',anonymous=True)

    # Subscribers
    rospy.Subscriber('rtk',rtk_fix,callback_rtk)
    rospy.Subscriber('cmd_vel_img', Twist, callback_cmd_vel_img)
    rospy.Subscriber('mission', MissionStage, callback_mission)
    rospy.Subscriber('abort_mission', Bool, callback_abort_mission)

    # Publishers
    start_map_pub = rospy.Publisher('start_map',Bool, queue_size=10)
    start_visual_servoing_pub = rospy.Publisher('start_visual', Bool, queue_size=10)
    save_mission_pub = rospy.Publisher('save_mission', UInt8, queue_size=10)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals
    save_map_pub = rospy.Publisher('save_map',Bool, queue_size=10)
    cmd_vel_pub.publish(Twist())

    # Getting parameter
    RATE = rospy.get_param("line_following/control/rate")
    rate = rospy.Rate(RATE)

    # Recovering Thresholds
    linear_threshold = rospy.get_param("line_following/control/linear_threshold")
    angular_threshold = rospy.get_param("line_following/control/angular_threshold")
    local_mission = rospy.get_param("line_following/control/local_mission")
    max_v = rospy.get_param("line_following/control/linear_velocity")
    max_omega = rospy.get_param("line_following/control/angular_velocity")
    gps_offset = rospy.get_param("line_following/control/gps_offset")
    edge_size = rospy.get_param("line_following/control/edge_size")

    # Recovering the control gains
    gains_line = rospy.get_param("line_following/control/gains_line")
    gains_polar = rospy.get_param("line_following/control/gains_polar")
    control_weight = rospy.get_param("line_following/control/control_weight")

    # Auxiliary variables
    in_edge = True

    while not rospy.is_shutdown():

        if not stage:
            print("Waiting new mission.",end="\r")

        else:
            # New version
            print(f"\nRunning mission {stage}")

            # Excluding the first line for mission 2
            local_map = copy.deepcopy(validate_map[1:] if len(validate_map) > 1 else validate_map)
            print(f"Local map:\n{local_map}\nValidate map:\n{validate_map}")
            abort = False

            for index, line in enumerate(local_map):

                # Getting the reference points
                mission.start_latitude = line.points[0].latitude
                mission.start_longitude = line.points[0].longitude
                line.points.pop(0)

                for i, p in enumerate(line.points):

                    d = geopy.distance.geodesic((mission.start_latitude, mission.start_longitude), (p.latitude, p.longitude)).m
                    print(f"Start: ({mission.start_latitude}, {mission.start_longitude})\n"
                          f"Stop (candidate): ({p.latitude}, {p.longitude})\n"
                          f"Distance: {d}")
                    if d >= edge_size:
                        mission.stop_latitude = p.latitude
                        mission.stop_longitude = p.longitude
                        input(f"Candidate defined:\n{p}\nDistance: {d}")
                        break

                in_mission = True

                # Defining the submission type as TO_START
                to_start = True

                # Setting the mission in the controller
                discovery_control.set_line(mission, local=True)

                # Setting the robot current pose as origin
                discovery_control.set_origin(copy.deepcopy(curr_pose))

                # Resetting the integral therm
                discovery_control.reset_integral()

                print(f"Mission Line {index}:\n{mission}")

                while in_mission and not abort:
                    # Starting the control signal variable
                    cmd_vel = Twist()

                    if to_start:
                        print(f"*** Mission {stage}: Going to first point of the line {index} ***")

                        # Computing the control signal based on robot position
                        cmd_vel, ang_err, rho = discovery_control.go_to_pose(curr_pose,
                                                                             destination="start",
                                                                             polar_gains=gains_polar,
                                                                             local_coord=local_mission)

                        print(f"*** Mission Status ***\n   Distance: {rho}\n   Angular Err: {ang_err}\n  In edge: {in_edge}")

                        # Stop condition
                        if abs(ang_err) <= angular_threshold and rho <= linear_threshold:
                            print("\nReach the stop condition.")
                            if stage == 2 and in_edge:
                                input("\nReach the line start point.\nGoing to crop start point...")
                                # Continue to the next start point
                                to_start = True

                                # Change the location to going outside the edge
                                in_edge = False

                                # Getting the new reference points
                                mission.start_latitude = mission.stop_latitude
                                mission.start_longitude = mission.stop_longitude
                                line.points.pop(0)

                                for i, p in enumerate(line.points):
                                    d = geopy.distance.geodesic((mission.start_latitude, mission.start_longitude),
                                                                (p.latitude, p.longitude)).m
                                    print(f"Start: ({mission.start_latitude}, {mission.start_longitude})\n"
                                          f"Stop (candidate): ({p.latitude}, {p.longitude})\n"
                                          f"Distance: {d}")
                                    if d >= edge_size:
                                        mission.stop_latitude = p.latitude
                                        mission.stop_longitude = p.longitude
                                        input(f"Second Candidate defined:\n{p}\nDistance: {d}")
                                        break

                            else:

                                # Stop TO START routine
                                to_start = False

                                # Reset the edge flag
                                in_edge = True

                                # Setting the new reference point to final point
                                mission.stop_latitude = line.points[-1].latitude
                                mission.stop_longitude = line.points[-1].longitude

                                # Setting the mission in the controller
                                discovery_control.set_line(mission, local=True)

                                # Start the visual_servoing controller
                                start_visual_servoing_pub.publish(True)

                                # Start saving map
                                start_map_pub.publish(True)

                                # Set all robot velocities to zero
                                cmd_vel = Twist()

                                print("The robot reached the start point.")


                    else:
                        print(f"*** Mission {stage}: Going to last point of the line {index} ***")

                        # Getting the signal from line follower controller
                        cmd_vel_lin, ang_err, rho = discovery_control.go_to_pose(curr_pose,
                                                                                 destination="stop",
                                                                                 w=control_weight[2],
                                                                                 line_gains=gains_line,
                                                                                 polar_gains=gains_polar,
                                                                                 local_coord=local_mission)

                        print(f"Mission State:\n   Distance: {rho}\n   Angular Err: {ang_err}")

                        # Creating the velocity variables
                        # Using both visual servoing (mostly) and GPS inside the talhao
                        if not math.isnan(cmd_vel_img.linear.x) and not math.isnan(cmd_vel_img.angular.z):
                            cmd_vel.linear.x = (control_weight[0]*cmd_vel_lin.linear.x + \
                                            (1-control_weight[0])*cmd_vel_img.linear.x)*(abs(np.tanh(rho)))
                            cmd_vel.angular.z = control_weight[1]*cmd_vel_lin.angular.z + \
                                            (1-control_weight[1])*cmd_vel_img.angular.z
                        else:
                            cmd_vel.linear.x = cmd_vel_lin.linear.x
                            cmd_vel.angular.z = cmd_vel_lin.angular.z

                        # Using only GPS while outside the talhao (°|°)
                        if rho < edge_size and stage == 2:
                            print("Inside the edge (bordadura) - Using only GPS")
                            cmd_vel.linear.x = (cmd_vel_lin.linear.x )*(abs(np.tanh(rho)))
                            cmd_vel.angular.z = cmd_vel_lin.angular.z

                        if abs(ang_err) <= angular_threshold and rho <= linear_threshold:
                            # Stop robot
                            cmd_vel = Twist()

                            # Stop visual servoing control signal
                            start_visual_servoing_pub.publish(False)

                            # Saving the map
                            save_mission_pub.publish(stage)

                            # Ending mission
                            in_mission = False

                            print("The robot reached the line end point.\nGoing to the next line.")


                    # Velocity threshold
                    cmd_vel.linear.x = cmd_vel.linear.x if abs(cmd_vel.linear.x) <= max_v else np.sign(
                        cmd_vel.linear.x) * max_v
                    # cmd_vel.linear.x = np.sign(cmd_vel.linear.x) * max_v

                    cmd_vel.angular.z = cmd_vel.angular.z if abs(cmd_vel.angular.z) <= max_omega else np.sign(
                        cmd_vel.angular.z) * max_omega

                    # Publishing the velocity command
                    cmd_vel_pub.publish(cmd_vel)

                    signal(SIGINT, exiting)
                    rate.sleep()

            save_map_pub.publish(True)

            # Assuming the mission is accomplished
            stage = 0

        #signal(SIGINT, exiting)
        rate.sleep()

#### Main ###
mission = Mission()
mission.start_mission = False
validate_map = []
robot_pose = Pose2D()
cmd_vel_img = Twist()
cmd_vel_lin = Twist()
abort = False
stage = 0
gps_offset = 0

# Creating the control object
discovery_control = LineControl()
curr_pose = Pose2D()

if __name__ == "__main__":

    main()

