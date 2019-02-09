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

#define GUI
class ardrone_object_tracking {

   public:
      
      // Initialization list
      ardrone_object_tracking():
         
         // Initialize image transport instance
         it(nH)
         
         // Initialize subscriber
         { 
            arImageSub = it.subscribe(arImageTopic, 1, &ardrone_object_tracking::imageCallback, this);
            tracker = cv::TrackerKCF::create();

         }

      void imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

         try {
            frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            
            // Flip to hsv
            frame = processImg(frame, 145, 50, 65);

            if (frameCount = 0 || frameCount > 10) {
                
                // Get roi
                roi = detectObject(frame);

                if (roi.height > 0) {
                    tracker.release();
                    tracker = cv::TrackerKCF::create();
                    tracker->init(frame, roi);
                    frameCount = 1;
                }

            } else {

                ok = tracker->update(frame, roi);

                if (ok) {


                }

            }
            frameCount++;

         #ifdef GUI
            cv::imshow("Node", frame); 
            cv::waitKey(30);
         #endif

         } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Image encoding error %s", e.what());
         }

      }

   private:
      const std::string arImageTopic = "/ardrone/front/image_raw";
      const std::string controlTopic = "";

      ros::NodeHandle nH;
      image_transport::ImageTransport it;
      
      // Image transport subscriber
      image_transport::Subscriber arImageSub;

      // Publisher
      ros::Publisher controlPub;
      
      // CV variables
      bool ok;
      int frameCount = 0;

      cv::Mat frame;
      cv::Rect2d roi;
      cv::Ptr<cv::TrackerKCF> tracker;

};


int main(int argc, char** argv) {
   
   std::string nodeName = "ardrone_object_tracking_node";
   ros::init(argc, argv, nodeName);

   ardrone_object_tracking ar;
   
   
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
