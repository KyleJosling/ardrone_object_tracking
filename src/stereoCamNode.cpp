// stereoCamNode
// might be replaced with standalone usb_cam ROS node
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <stdio.h>

// Constants
const int CAM_FPS = 30;                                 // Camera capture rate
const std::string LEFT_TOPIC_NAME = "/image_left";       // Left topic name
const std::string RIGHT_TOPIC_NAME = "/image_right";     // Right topic name
const int ADVERTISE_QUEUE_SIZE = 1;                     // Advertising queue size
const int PUBLISH_SPEED = 1;                            // Loop rate

int main(int argc, char** argv) {
    
    // Window for GUI
    #ifdef GUI
        std::string windowName = "stereoCam";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    #endif

    // OpenCV capture object
    cv::VideoCapture cap(0);

    // CV frame object
    cv::Mat frame;
    
    if (!cap.isOpened()) {
        return 0;
    }
    
    // Initialize ROS, node handle, publisher
    ros::init(argc, argv, "stereoCamNode"); // TODO
    ros::NodeHandle nodeHandle;
    
    // Create image transport instance
    image_transport::ImageTransport imTransportInst(nodeHandle);

    // Initialize publishers
    image_transport::Publisher leftPub = imTransportInst.advertise(LEFT_TOPIC_NAME, ADVERTISE_QUEUE_SIZE);
    image_transport::Publisher rightPub = imTransportInst.advertise(RIGHT_TOPIC_NAME, ADVERTISE_QUEUE_SIZE);

    // Declare msgs to publish on topics
    sensor_msgs::ImagePtr left_image_msg;
    sensor_msgs::ImagePtr right_image_msg;

    // Loop rate (experimental value)
    ros::Rate loop_rate(PUBLISH_SPEED);

    // Camera calibration parameters TODO
    

    while (ros::ok()) {

        // cap >> frame;
        frame = cv::imread("/home/kylejosling/Downloads/DataSets/Scrape/JPEG/16728198407_6c51645269_b.jpg");

        // Pack messages TODO will eventually be two different images
        left_image_msg  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        // right_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        
        leftPub.publish(left_image_msg);
        // rightPub.publish(right_image_msg);

        //ros::spinOnce();
        loop_rate.sleep();
        
        #ifdef GUI
        cv::imshow(windowName, frame);
        cv::waitKey(30);
        #endif


    }

    #ifdef GUI
    cv::destroyWindow(windowName);
    #endif
    
    return 0;
}
