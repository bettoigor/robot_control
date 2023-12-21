#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for line following control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 3.0
Date: 6-20-2023
"""

import rospy
from utils.discovery_lib import Discovery


if __name__ == "__main__":

    print("\n *** Starting Discovery Control Node v 3.0 *** \n\n")

    # Creating control object
    discovery = Discovery()

    # Getting parameter
    discovery.rate = rospy.get_param("line_following/control/rate")

    # Recovering Thresholds
    discovery.linear_threshold = rospy.get_param("line_following/control/linear_threshold")
    discovery.angular_threshold = rospy.get_param("line_following/control/angular_threshold")
    discovery.local_mission = rospy.get_param("line_following/control/local_mission")
    discovery.max_v = rospy.get_param("line_following/control/linear_velocity")
    discovery.max_omega = rospy.get_param("line_following/control/angular_velocity")
    discovery.gps_offset = rospy.get_param("line_following/control/gps_offset")
    discovery.edge_size = rospy.get_param("line_following/control/edge_size")

    # Recovering the control gains
    discovery.gains_line = rospy.get_param("line_following/control/gains_line")
    discovery.gains_polar = rospy.get_param("line_following/control/gains_polar")
    discovery.control_weight = rospy.get_param("line_following/control/control_weight")


    # Setting control parameters
    discovery.run()

