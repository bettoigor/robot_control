/*
    File containing methods used to control
    the robot.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 1.0
    Date: 10-3-2022
*/

#include "librobotcontrol.h"

/* 
    Constructors 
*/
RobotControl::RobotControl(ros::NodeHandle nh)
{
    // Creating publishers
    pubSpeed = nh.advertise<robot_control::Speed>("speed",1000);
    subRtk = nh.subscribe("rtk",1000,
                            &RobotControl::callbackRtk, this);
    subCmdVel = nh.subscribe("cmd_vel",1000,
                            &RobotControl::callbackCmdVel, this);

    // Setting control parameters

    ros::param::get("control/phisical_params/wheel_separation",WHEEL_SEPARATION);
    ros::param::get("control/phisical_params/radius_left_wheel",LEFT_WHEEL);
    ros::param::get("control/phisical_params/radius_right_wheel",RIGHT_WHEEL);
    ros::param::get("control/phisical_params/max_speed",MAX_SPEED);
    ros::param::get("control/phisical_params/motor_gain_l",MOTOR_GAIN_L);
    ros::param::get("control/phisical_params/motor_gain_r",MOTOR_GAIN_R);
    ros::param::get("control/control/force_cmd_vel",FORCE_CMD_VEL);
    ros::param::get("control/communication/type",COMM_TYPE);
    ros::param::get("control/communication/udp_port",PORT);

    // Creating UDP communication port
    pubSpeedUDP.setServer(PORT);

}

RobotControl::~RobotControl()
{
    
    // ClassDestrictor
}

/*
    Getters
*/
geometry_msgs::Twist RobotControl::getCmdVel()
{

    return cmdVel;
}

int RobotControl::getGpsQuality()
{

    return gpsQuality;
}

bool RobotControl::getPubState(int type=0)
{
    bool state;

    switch(type)
    {
        case 0:
            state = isCmdVel;
        break;

        case 1:
            state = isRtkFix;
        break;

        case 2:
            state = FORCE_CMD_VEL;
        break;

        default:
            state = isCmdVel;
        break;    
    }


    return state;
}


/*
    Workers
*/
// Callbacks
void RobotControl::callbackRtk(solarbot::rtk_fix msg)
{
    
    gpsQuality = msg.quality;
    isRtkFix = true;
}

void RobotControl::callbackCmdVel(geometry_msgs::Twist msg)
{

    cmdVel = msg;
    isCmdVel = true;
}

// Actions
vector<float> RobotControl::generateSpeed(geometry_msgs::Twist cmdVel)
{
    
    vector<float> speed;
    float v = cmdVel.linear.x;
    float omega = cmdVel.angular.z;
    float phiL,phiR;
    float L = WHEEL_SEPARATION/2;


    phiL = ((v - (L*omega))/LEFT_WHEEL);
    phiR = ((v + (L*omega))/RIGHT_WHEEL);

    phiL = ((v - (L*omega))/LEFT_WHEEL)*MOTOR_GAIN_L;
    phiR = ((v + (L*omega))/RIGHT_WHEEL)*MOTOR_GAIN_R;


    // Speed limitation
    if (fabs(phiL) > MAX_SPEED)
        phiL = (phiL/fabs(phiL))*MAX_SPEED;

    if (fabs(phiR) > MAX_SPEED)
        phiR = (phiR/fabs(phiR))*MAX_SPEED;
    
    speed.push_back(phiL);
    speed.push_back(phiR);

    return speed;
}

void RobotControl::pubVelocity()
{
    vector<float> speed=this->generateSpeed(cmdVel);
    robot_control::Speed msgSpeed;
    std::string msg = "MOTO," +
                      std::to_string(static_cast<int>(0)) + "," +
                      std::to_string(static_cast<int>(0));

    if (((isRtkFix and gpsQuality == 4) or FORCE_CMD_VEL) and isCmdVel)
    {
        // For ROS publisher
        msgSpeed.left_speed = speed[0];
        msgSpeed.right_speed = speed[1];

        // For UDP publisher
        msg = "MOTO," +
              std::to_string(static_cast<int>(speed[0])) + "," +
              std::to_string(static_cast<int>(speed[1]));
    }

    // Selecting publish mode
    if (COMM_TYPE == 0)
    {
        pubSpeed.publish(msgSpeed);

    }
    else
    {
        pubSpeedUDP.send(msg);

        std::cout << "\nPublish message using UDP protocol: " << msg
                    << "\nPort: " << PORT
                    << "\nComm Type: " << COMM_TYPE << std::endl;

    }
    isCmdVel = false;
    isRtkFix = false;
}
