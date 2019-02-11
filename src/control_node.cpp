#include "ros/ros.h"

#include "pid.h"

#include <iostream>
#include <stdio.h>

#include <ardrone_object_tracking/ObjectMsg.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class controller {

    public:
        
        controller():
        
        // Initialize PID controller
        // ( double dt, double max, double min, double Kp, double Kd, double Ki );
        yawPid(0.1, 320, -320, 0.702, 4.9, 0.00006) { 

            // Subscriber for object location
            objectSub = nH.subscribe(objectTopic, 1, &controller::objectCallback, this);

            // Publishers commands
            controlPub = nH.advertise<geometry_msgs::Twist>(controlTopic, 1000);
            takeoffPub = nH.advertise<std_msgs::Empty>(takeoffTopic, 1000);
            landingPub = nH.advertise<std_msgs::Empty>(landingTopic, 1000);
            
            startupRoutine();

        }

        ~controller() {
            ROS_INFO("Destructor called");
            std_msgs::Empty msg; 
            landingPub.publish(msg);
        }
        
        void objectCallback(const ardrone_object_tracking::ObjectMsg::ConstPtr &objectPos) {

            yawPVar = objectPos->x;
            yawOutput = (yawPid.calculate(yawSVar, yawPVar));
            // Normalize
            yawOutput = yawOutput/320;
            ROS_INFO("Received object position: ");
            ROS_INFO(" X: %f , Y: %f, H : %f \n", objectPos->x, objectPos->y, objectPos->height);
            ROS_INFO("Yaw output : %f", yawOutput);
            
            sendCommand( 0, 0, 0, yawOutput);
        }
        
        // Send command to AR Drone
        void sendCommand( int x, int y, int z, int yaw) {
            
            // Linear and angular components
            geometry_msgs::Vector3 linear; 
            geometry_msgs::Vector3 angular;
            geometry_msgs::Twist command;
            
            // Pack and publish command
            linear.x = x;
            linear.y = y;
            linear.z = z;

            angular.x = 0;
            angular.y = 0;
            angular.z = yaw;

            command.linear = linear;
            command.angular = angular;

            controlPub.publish(command);

        }

        void startupRoutine() {
            
            ROS_INFO("Startup routine initiated");

            // Sleep and then take off
            usleep(5000000);
            // Set to hover mode
            sendCommand( 0, 0, 0, 0);
            std_msgs::Empty msg; 
            takeoffPub.publish(msg);

        }

    private:
        const std::string takeoffTopic = "/ardrone/takeoff";
        const std::string landingTopic = "/ardrone/land";
        const std::string objectTopic = "/object";
        const std::string controlTopic = "/cmd_vel";

        ros::NodeHandle nH;

        // Subscriber
        ros::Subscriber objectSub;
        
        // Publisher
        ros::Publisher controlPub;
        ros::Publisher takeoffPub;
        ros::Publisher landingPub;

        // PID controller variables
        PID yawPid;

        int yawSVar = 320;
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
