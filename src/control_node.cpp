#include <ros/ros.h>

#include <signal.h>
#include <iostream>
#include <stdio.h>

#include "pid.h"

#include <ardrone_object_tracking/ObjectMsg.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#define NO_FLY

sig_atomic_t volatile g_request_shutdown = 0;

class controller {

    public:
        
        controller():
        
        // Initialize PID controllers
        // ( double dt, double max, double min, double Kp, double Kd, double Ki );
        yawPid(0.1, 1.0, -1.0, 0.702, 4.9, 0.00006),
        pitchPid(0.1, 1.0, -1.0, 0.702, 4.9, 0.00006) { 

            // Subscriber for object location
            objectSub = nH.subscribe(objectTopic, 1, &controller::objectCallback, this);

            // Publishers commands
            controlPub = nH.advertise<geometry_msgs::Twist>(controlTopic, 1000);
            takeoffPub = nH.advertise<std_msgs::Empty>(takeoffTopic, 1000);
            landingPub = nH.advertise<std_msgs::Empty>(landingTopic, 1000);
            
            startupRoutine();
        }

        ~controller() {
            sendCommand( 0, 0, 0, 0);
            std_msgs::Empty msg; 
            landingPub.publish(msg);
        }
        

        void objectCallback(const ardrone_object_tracking::ObjectMsg::ConstPtr &objectPos) {

            yawPVar = objectPos->x;
            yawOutput = (yawPid.calculate(yawSVar, yawPVar));
            pitchPVar = objectPos->height;
            pitchOutput = (pitchPid.calculate(pitchSVar, pitchPVar));

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
            #ifndef NO_FLY
            controlPub.publish(command);
            #endif

        }

        void startupRoutine() {
            
            ROS_INFO("Startup routine initiated");
            
            #ifndef NO_FLY
            // Sleep and then take off
            usleep(5000000);
            // Set to hover mode
            sendCommand( 0, 0, 0, 0);
            std_msgs::Empty msg; 
            takeoffPub.publish(msg);
            usleep(2000000);
            #endif

        }

        void shutdownRoutine() {
            
            ROS_INFO("Landing");

            // Set to hover mode
            sendCommand( 0, 0, 0, 0);
            std_msgs::Empty msg; 
            landingPub.publish(msg);

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
        PID pitchPid;

        int yawSVar = 320;
        int yawPVar;
        int pitchSVar = 220;
        int pitchPVar;

        double yawOutput;
        double pitchOutput;

};

void sigintHandler (int sig) {
    g_request_shutdown = 1;
}

int main(int argc, char** argv) {

    std::string nodeName = "control_node";
    ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
    signal(SIGINT, sigintHandler);

    controller c;
    
    while (!g_request_shutdown) {
        ros::spinOnce();
    }

    c.shutdownRoutine(); 
    ros::shutdown();
    return 0;

}
