/*
    Header file containing classess used to control
    the robot.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 1.0
    Date: 10-3-2022
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <fstream>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <cmath>
#include <robot_control/Speed.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <solarbot/rtk_fix.h>
#include <stdlib.h>
#include <stdio.h>
#include "UDPSocket.h"

using namespace std;

class RobotControl
{//Class begin

    /*
        Class to handle robot control task
    */

private:

    // Creating Publishers and Subscribers
    ros::Publisher pubSpeed;
    ros::Subscriber subRtk;
    ros::Subscriber subCmdVel;

    // Creating UDP Publisher
    UDPSocket pubSpeedUDP;

    // Creating control variables
    geometry_msgs::Twist cmdVel;
    robot_control::Speed speed;
    solarbot::rtk_fix rtkFix;
    int gpsQuality = -1;
    bool isCmdVel = false;
    bool isRtkFix = false;

    // Robot control parameters (Constants)
    float 
        WHEEL_SEPARATION,
        LEFT_WHEEL,
        RIGHT_WHEEL,
        MAX_SPEED,
        MOTOR_GAIN_L,
        MOTOR_GAIN_R;

    bool
        FORCE_CMD_VEL;

    // Communication parameters (for UDP commnunication mode)
    int
        COMM_TYPE;

    int
        PORT;




public:

    // Constructors
    RobotControl(ros::NodeHandle nh);
    
    ~RobotControl();

    // Getters
    geometry_msgs::Twist getCmdVel();

    int getGpsQuality();

    bool getPubState(int type);

    // Workers
    void callbackCmdVel(geometry_msgs::Twist msg);

    void callbackRtk(solarbot::rtk_fix msg);

    vector<float> generateSpeed(geometry_msgs::Twist cmdVel);

    void pubVelocity();

    void run();

    // Communication
    int startUDP();


};//Class end