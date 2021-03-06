#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/tracking/tracking.hpp>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <stdio.h>

#include "ardrone_object_tracking/detectObject.hpp"
#include "ardrone_object_tracking/ObjectMsg.h"


class ardrone_object_tracker {

    public:

        // Initialization list
        ardrone_object_tracker():

        // Initialize image transport instance
        it(nH)

        { 
            // Initialize publisher 
            objectPub = nH.advertise<ardrone_object_tracking::ObjectMsg>(objectTopic, 1000);
            // Initialize subscriber
            arImageSub = it.subscribe(arImageTopic, 1, &ardrone_object_tracker::imageCallback, this);
            tracker = cv::TrackerKCF::create();
            setParams();

        }

        void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

            try {

                static int frameCount = 0;
                cv::Mat original;
                original = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

                // Flip to hsv
                frame = processImg(original, hue, sat, val, width);

                if (frameCount == 0 || frameCount > 10) {

                    // Get roi
                    roi = detectObject(frame);
                    ROS_INFO("Getting object");

                    if (roi.height > 0) {
                        ROS_INFO("Height > 0");
                        tracker.release();
                        tracker = cv::TrackerKCF::create();
                        tracker->init(frame, roi);
                        frameCount = 1;
                    }

                } else {

                    ok = tracker->update(frame, roi);
                    
                    // If everything is cool, publish message
                    if (ok) { 
                        msg.x = roi.x + (roi.width/2);
                        msg.y = roi.y + (roi.height/2);
                        msg.height = roi.height;
                        objectPub.publish(msg);
                    }

                    frameCount++;
                }


            #ifdef GUI
                cv::rectangle(original, roi.tl(),roi.br(),cv::Scalar(0,255,0),2,8);
                cv::imshow("Node", original); 
                cv::imshow("Node1", frame); 
                cv::waitKey(30);
            #endif

            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("Image encoding error %s", e.what());
            }

        }
        
        // Set parameters using yaml file
        void setParams() {

            cv::FileStorage fs("/home/kylejosling/ros_ws/src/ardrone_object_tracking/include/ardrone_object_tracking/vision-config.yml", cv::FileStorage::READ);

            if (!fs.isOpened())
            {
                ROS_ERROR("Failed to open config file.");
                return;
            }

            // Set horizontal cutoff border
            fs["hue"] >> hue;
            fs["sat"] >> sat;
            fs["val"] >> val;
            fs["width"] >> width;

            // Close the YAML file
            fs.release();
        }


    private:
        const std::string arImageTopic = "/ardrone/front/image_raw";
        const std::string objectTopic = "/object";

        ros::NodeHandle nH;
        image_transport::ImageTransport it;

        // Image transport subscriber
        image_transport::Subscriber arImageSub;

        // Publisher
        ros::Publisher objectPub;
        ardrone_object_tracking::ObjectMsg msg;

        // CV variables
        bool ok;
        int hue, sat, val, width;

        cv::Mat frame;
        cv::Rect2d roi;
        cv::Ptr<cv::TrackerKCF> tracker;

};


int main(int argc, char** argv) {

    std::string nodeName = "ardrone_object_tracking_node";
    ros::init(argc, argv, nodeName);

    ardrone_object_tracker ar;


    #ifdef GUI
        // OpenCV declares
        std::string windowName = "ardrone";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::startWindowThread();
    #endif

    ros::spin();

    #ifdef GUI
        cv::destroyWindow(windowName);
    #endif
    return 0;
}
