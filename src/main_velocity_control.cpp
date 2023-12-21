/*
    Main code of robot control node.
    This node receives cmd_vel messagens, with
    linear and angular velocities and converts to 
    wheel angular speed L and R and publishes to the
    low level control node running in the solarbot 
    control program.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 2.0
    Date: 10-3-2022
*/


#include "librobotcontrol.h"


/*************** MAIN **************/
int main(int argc, char** argv)
{// Starting the main function for wheel speed control node

    // List of used variables
    int rateHz;

    // Initializing the ROS structure
    ros::init(argc, argv, "speed_control_node");
    ros::NodeHandle nh;
    ROS_INFO("*** ROS node successfully started! ***");
    
    // Loading and setting the control parameters
    cout << "\nGetting control parameters... ";
    ros::param::get("control/control/rate",rateHz);
    ros::Rate loop_rate(rateHz);
    cout <<"Done!";

    // Creating control object
    cout << "\nCreating control object... ";
    RobotControl control(nh);
    cout <<"Done!";


    // Starting the ROS main loop
    cout << "\nStarting the ros loop...\n";
    ROS_INFO(" *** Velocity control node successfully activated! ***");

    while (nh.ok())
    {

        // Just for monitoring purpose
        if (control.getPubState(0) and 
            ((control.getPubState(1) and control.getGpsQuality() == 4) or control.getPubState(2)))
        {
            vector<float>  speed =  control.generateSpeed(control.getCmdVel());
            cout << "\nGPS Quality: "<< control.getGpsQuality()
                 << " Linear Vel: "     << control.getCmdVel().linear.x
                 << " Angular Vel: " << control.getCmdVel().angular.z
                 << " Wheel speed L:" << speed[0] << " R: " << speed[1];            
        }
        else
        {
            cout << "\n\nSpeed topic is not published due to:";
            if (!control.getPubState(0))
                cout << "\n * cmd_vel topic is not yet published!";
            if (!control.getPubState(1))
                cout << "\n * GPS topic is not yet published!";
            else if (control.getGpsQuality() != 4)
                cout << "\n * low quality GPS signal!";

        }

        
        // Publishing wheel speed
        control.pubVelocity();

        // ROS control action
        ros::spinOnce();
        loop_rate.sleep();

    }
}