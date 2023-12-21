#include "librobotcontrol.h"


/*************** MAIN **************/
int main(int argc, char** argv)
{// Starting the main function for wheel speed control node

    /*
        List of used variables
    */
    int
        rateHz;

    float phiL = 0;
    float phiR = 0;

    //bool

    robot_control::MotorStatus motorStausData;

    /*
        Initializing the ROS structure
    */ 
    ros::init(argc, argv, "speed_control_node");
    ros::NodeHandle nh;
    ROS_INFO("ROS node successfully started!");
    
    /*
        Loading parameters
    */
    ROS_INFO("Getting control parameters...");
    ros::param::get("control/control/rate",rateHz);


    // Creating publishers
    ros::Publisher motor_status = nh.advertise<robot_control::MotorStatus>("motor_status",1000);
    


    // Control Parameters
    ros::Rate loop_rate(rateHz);


    // Setting speed values
    if (argc == 3)
    {
        phiL = atol(argv[1]);
        phiR = atol(argv[2]);

        //phiL = (float)strtod(argv[1],NULL);
        //phiR = (float)strtod(argv[2],NULL);
    }


    // Starting the ROS main loop
    cout << " Done!\nStarting the ros loop...\nRunning ROS: " << nh.ok() << endl;

    while (nh.ok())
    {
        
        ROS_INFO(" *** Speed control node successfully activated! ***");

        // Setting speed values
        motorStausData.left_speed = phiL;
        motorStausData.right_speed = phiR;

        // Publish messages
        cout << "\nPublishing Motor Status message...";
        motor_status.publish(motorStausData);

        // ROS control action
        //ros::spinOnce();
        loop_rate.sleep();

    }
}