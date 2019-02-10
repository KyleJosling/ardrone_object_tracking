#include "ros/ros.h"

#include "pid.h"

#include <iostream>
#include <stdio.h>

#include <ardrone_object_tracking/ObjectMsg.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

class controller {

    public:
        
        controller():
        
        // Initialize PID controller
        // ( double dt, double max, double min, double Kp, double Kd, double Ki );
        yawPid(0.1, 500, -500, 0.702, 4.9, 0.00006)
        { 

            // Subscriber for object location
            objectSub = nH.subscribe(objectTopic, 1, &controller::objectCallback, this);

            // Publishers commands
            controlPub = nH.advertise<ardrone_object_tracking::ObjectMsg>(controlTopic, 1000);
            takeoffPub = nH.advertise<std_msgs::Empty>(takeoffTopic, 1000);
            landingPub = nH.advertise<std_msgs::Empty>(landingTopic, 1000);

            usleep(5000000);
            std_msgs::Empty msg; 
            ROS_INFO("SLEEPING");
            takeoffPub.publish(msg);
            usleep(5000000);
            ROS_INFO("AWAKE");
            landingPub.publish(msg);
        }

        void objectCallback(const ardrone_object_tracking::ObjectMsg::ConstPtr &objectPos) {
            yawOutput = (yawPid.calculate(yawSVar, yawPVar) + 1500);
            ROS_INFO("Received object position");

        }

    private:
        const std::string takeoffTopic = "/ardrone/takeoff";
        const std::string landingTopic = "/ardrone/land";
        const std::string objectTopic = "object";
        const std::string controlTopic = "/ardrone/cmd_vel";

        ros::NodeHandle nH;

        // Subscriber
        ros::Subscriber objectSub;
        
        // Publisher
        ros::Publisher controlPub;
        ros::Publisher takeoffPub;
        ros::Publisher landingPub;

        // PID controller variables
        PID yawPid;

        int yawSVar = 352/2;
        int yawPVar;

        double yawOutput;

};


int main(int argc, char** argv) {


    std::string nodeName = "control_node";
    ros::init(argc, argv, nodeName);

    controller c;

    ros::spin();

    return 0;

}
