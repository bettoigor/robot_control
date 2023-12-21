#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for line following control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 8-5-2022
"""

import rospy, time, angles, copy, math, sys, os, rostopic, geopy.distance, copy
from utils.control_lib import *
from robot_control.msg import Mission
from nav_msgs.msg import Odometry
from solarbot.msg import rtk_fix
from geometry_msgs.msg import Pose2D,Twist
from signal import signal, SIGINT
from datetime import datetime
from solix_bringup.msg import MissionStage
from std_msgs.msg import UInt8, Bool, Empty, String

def callback_mission_stage_one(data):

    global mission
    global stage

    stage = 1

    mission.start_mission = data.start
    mission.start_latitude = data.lines[0].points[0].latitude
    mission.start_longitude = data.lines[0].points[0].longitude
    mission.stop_latitude = data.lines[0].points[1].latitude
    mission.stop_longitude = data.lines[0].points[1].longitude


    print(f"Mission data:\n{mission}")


def callback_mission_stage_two(data):

    global mission_two
    global stage

    stage = 2


    mission_two = data

def callback_mission(data):

    global mission
    global discovery_control
    global curr_pose
    
    mission = data
    #discovery_control.set_origin(curr_pose)

    print(f"Mission successfully received!!\n{mission}")

def callback_rtk(data):

    global curr_pose

    gps_data = data

    lat = gps_data.latitude
    lon = gps_data.longitude
    heading = np.deg2rad(gps_data.heading+1.85)
    quality = gps_data.quality
    #curr_pose = (lat,lon,heading)

    curr_pose.x = lat
    curr_pose.y = lon
    curr_pose.theta = heading

def callback_cmd_vel_img(data):
    
    global cmd_vel_img

    cmd_vel_img = data

def exiting(signal_received, frame):

    # Publishers
    global cmd_vel_pub

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

    # Initializing ros node
    rospy.init_node('line_following_node',anonymous=True)

    # Subscribers
    rospy.Subscriber('rtk',rtk_fix,callback_rtk)
    rospy.Subscriber('mission',Mission,callback_mission)
    rospy.Subscriber('cmd_vel_img', Twist, callback_cmd_vel_img)
    rospy.Subscriber('mission_1', MissionStage, callback_mission_stage_one)
    rospy.Subscriber('mission_2', MissionStage, callback_mission_stage_two)

    # Publisher
    start_map_pub = rospy.Publisher('start_map',Bool, queue_size=10)
    start_visual_servoing_pub = rospy.Publisher('start_visual', Bool, queue_size=10)
    save_mission_pub = rospy.Publisher('save_mission', String, queue_size=10)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals
    cmd_vel_pub.publish(Twist())

    # Getting parameter
    RATE = rospy.get_param("line_following/control/rate")
    rate = rospy.Rate(RATE)

    # Recovering Thresholds
    linear_threshold = rospy.get_param("line_following/control/linear_threshold")
    angular_threshold = rospy.get_param("line_following/control/angular_threshold")
    local_mission = rospy.get_param("line_following/control/local_mission")
    
    # Initializing the control variables
    end_distance = 0
    in_mission = False
    to_start = True

    while not rospy.is_shutdown():
        # Getting the velicity limits
        max_v = rospy.get_param("line_following/control/linear_velocity")
        max_omega = rospy.get_param("line_following/control/angular_velocity")
        
        # Recovering the control gains
        gains_line = rospy.get_param("line_following/control/gains_line")
        gains_polar = rospy.get_param("line_following/control/gains_polar")
        control_wheigth = rospy.get_param("line_following/control/control_wheight")

        # Monitoring the mission status and requisition
        if mission.start_mission and not in_mission:
            # Setting mission parameters
            print(f"\nMission started! \n{mission}")
            in_mission = True
            to_start = True
            mission.start_mission = False
            discovery_control.set_line(mission,local=True)
            origin_pose = copy.deepcopy(curr_pose)
            discovery_control.set_origin(origin_pose)
            
            point = (mission.start_latitude,mission.start_longitude)
            dist = geopy.distance.geodesic((discovery_control.origin.x,discovery_control.origin.y), point).m
        
            input(f"Robot zero position:\
                 \n{discovery_control.origin}\
                 \nRobot current pose:\
                 \n{curr_pose}\
                 \n Distance: {round(dist,3)} m")
        
        else:
            print(f"Waiting new mission...",end="\r")
            #print(f"Robot current position:\
            #      \n{curr_pose}")
        

        # Starting the mission
        if in_mission:
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print(f"***** Mission control time: {current_time} *****")
            print(f"Velocity commmand from image node:\
                  \n v: {cmd_vel_img.linear.x}\
                  \n omega: {cmd_vel_img.angular.z}\
		  \n Disantance: {round(dist,3)}")

            # Getting distances
            start_distance,stop_distance = discovery_control.get_point_distance(curr_pose,start=True,stop=True,local=local_mission)
            
            # Starting the first task: Go to start position
            if to_start:
                print("\n*** To Start Point action...\n")                
                
                dist = geopy.distance.geodesic(discovery_control.start, (curr_pose.x,curr_pose.y)).m
                #print(f"Distance to start point:\
                #    \n {round(dist,3)} m")
                
                # Getting the control signal 
                #cmd_vel,ang_err = discovery_control.go_to_start(curr_pose,gains_polar,local_coord=local_mission)
                cmd_vel,ang_err = discovery_control.go_to_pose(curr_pose,destination="start",polar_gains=gains_polar,local_coord=local_mission)
                print(f"Raw cmd_vel:\n   v: {cmd_vel.linear.x}\
			\n   omega: {cmd_vel.angular.z}\
			\n Start dis: {start_distance}")
                
                # Monitoring whether the action has completed
                if abs(ang_err) <= angular_threshold and \
                    abs(start_distance - 0.5) < linear_threshold:
                    cmd_vel = Twist()
                    to_start = False
                    start_map_pub.publish(True)
                    start_visual_servoing_pub.publish(True)
                    input(f"To Start Point action finished!\
			  \nAng Err: {ang_err}, Dist: {start_distance}")

            # Starting the second task: go to stop/end point
            else:

                print("\n*** To End Point action...\n")
                
                # Getting the signal from line follower controller
                v = max_v*(abs(np.tanh(stop_distance)))
                #cmd_vel_lin,ang_err = discovery_control.go_to_stop(v,curr_pose,w=control_wheigth[2],line_gains=gains_line,polar_gains=gains_polar,local_coord=local_mission)
                cmd_vel_lin,ang_err = discovery_control.go_to_pose(curr_pose,destination="stop",v_cruise=v,w=control_wheigth[2],line_gains=gains_line,polar_gains=gains_polar,local_coord=local_mission)
                
                # Merging the control signals
                cmd_vel = Twist()
                cmd_vel.linear.x = (control_wheigth[0]*cmd_vel_lin.linear.x + \
                                    (1-control_wheigth[0])*cmd_vel_img.linear.x)*(abs(np.tanh(stop_distance)))
                cmd_vel.angular.z = control_wheigth[1]*cmd_vel_lin.angular.z + \
                                    (1-control_wheigth[1])*cmd_vel_img.angular.z

                if (abs(stop_distance - 0.5) <= linear_threshold) and \
                      (ang_err < angular_threshold):
                    in_mission = False
                    cmd_vel = Twist()
                    save_mission_pub.publish(stage_type[stage])
                    start_visual_servoing_pub.publish(False)
                    print("To End Point action finished!")
                    time.sleep(2) 

            # Velocity limitation
            cmd_vel.linear.x = cmd_vel.linear.x if abs(cmd_vel.linear.x) <= max_v else np.sign(cmd_vel.linear.x)*max_v
            cmd_vel.angular.z = cmd_vel.angular.z if abs(cmd_vel.angular.z) <= max_omega else np.sign(cmd_vel.angular.z)*max_omega
            cmd_vel_pub.publish(cmd_vel)


            print(f"*** Task control *** Mode: {local_mission}\
                  \n To Start action: {to_start}\
                  \n Distances:\
                  \n  Start: {start_distance}\
                  \n  Stop: {stop_distance}\
                  \n  Nomalized: {np.tanh(stop_distance)}\
                  \n  Ang Error (deg): {np.deg2rad(ang_err)}\
                  \n Line Gains: {gains_line}\
                  \n Polar Gains: {gains_polar}\
                  \n Control signal:\
                  \n  v: {cmd_vel.linear.x}\
                  \n  omega: {cmd_vel.angular.z}\n\n")

        else:
            cmd_vel_pub.publish(Twist())


        #signal(SIGINT, exiting)
        rate.sleep()

#### Main ###
mission = Mission()
mission_one = MissionStage()
mission_two = MissionStage()
mission.start_mission = False
robot_pose = Pose2D()
cmd_vel_img = Twist()
cmd_vel_lin = Twist()
stage_type = {
    0: "stage_0",
    1: "stage_1",
    2: "stage_2"
}
stage = 0

# Creating the control object
discovery_control = LineControl()
curr_pose = Pose2D()

if __name__ == "__main__":

    main()

