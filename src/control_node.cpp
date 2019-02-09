#include "ros/ros.h"

#include <iostream>
#include <stdio.h>

#include <ardrone_object_tracking/ObjectMsg.h>

class controller {

    public:
        
        controller() { 
            objectSub = nH.subscribe(objectTopic, 1, &controller::objectCallback, this);
        }

        void objectCallback(const ardrone_object_tracking::ObjectMsg::ConstPtr &objectPos) {

            ROS_INFO("Received object position");
        }

    private:
        const std::string takeoffTopic = "'/ardrone/takeoff";
        const std::string landTopic = "/ardrone/land";
        const std::string objectTopic = "object";
        const std::string controlTopic = "/ardrone/cmd_vel";

        ros::NodeHandle nH;

        // Subscriber
        ros::Subscriber objectSub;
        
        // Publisher
        ros::Publisher controlPub;

};


int main(int argc, char** argv) {


    std::string nodeName = "control_node";
    ros::init(argc, argv, nodeName);

    controller c;

    ros::spin();

    return 0;

}
