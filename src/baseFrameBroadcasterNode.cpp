#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "baseFrameBroadcaster");
    ros::NodeHandle n;

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    ros::Rate rate(10.0);
    
    while (n.ok()) {
        
        transform.setOrigin(tf::Vector3(0,0,0));
        transform.setRotation(tf::Quaternion(0,0,0,0));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root", "world"));
    }

    return 0;
}
