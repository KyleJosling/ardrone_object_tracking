// testNode
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
const std::string LEFT_TOPIC_NAME = "raw_left_image";   // Left topic name
const std::string RIGHT_TOPIC_NAME = "raw_right_image"; // Right topic name
const int ADVERTISE_QUEUE_SIZE = 1;                     // Advertising queue size
const int PUBLISH_SPEED = 1;                            // Loop rate



class stemDetector {


    public:

        // Initialization list
        stemDetector(): 

        it(nH) {




        }


        // Test stem detection function
        void imageCallback(cv::Mat &src, cv::Rect2d roi) {
            
            // Extend the bounding box by 1/10 of the original size in each direction
            int oldY = roi.y;
            roi.y = roi.y - roi.height/10;
            roi.height = oldY - roi.y;
            // roi.width = roi.width/2;

            src = src(roi);

            cv::Mat dst;
            cv::Mat srcGray;
            cv::Mat detectedEdges;
            cv::cvtColor(src, srcGray, CV_BGR2GRAY);
            cv::blur( srcGray, detectedEdges, cv::Size(5,5) );

            cv::Canny(srcGray, dst, 10, 200, 3);

            // Copy edges to the images that will display the results in BGR
            // Hold the results of the detection
            std::vector<cv::Vec2f> lines;
            cv::HoughLines(dst, lines, 1, CV_PI/180, 15, 0, 0); // runs the actual detection
            std::cout << lines.size() << std::endl;
            
            // Find parallel lines that are stem like
            for( size_t i = 0; i < lines.size(); i++ ) {
                float rho1 = lines[i][0];
                float theta1 = lines[i][1];

                for( size_t j = i+1; j < lines.size(); j++ ) {

                    float rho2 = lines[j][0];
                    float theta2 = lines[j][1];

                    // We expect the stem angle to be less than 30 degrees
                    if (theta1 < CV_PI/6 && 
                        theta2 < CV_PI/6 && 
                        std::abs(theta1 - theta2) < 0.5 && 
                        std::abs(rho1 - rho2)  >  2) {
                        std::cout << "Stem detected" << std::endl;
                        
                        cv::Point pt1, pt2;
                        double a = std::cos(theta1), b = std::sin(theta1);
                        double x0 = a*rho1, y0 = b*rho1;
                        pt1.x = cvRound(x0 + 1000*(-b));
                        pt1.y = cvRound(y0 + 1000*(a));
                        pt2.x = cvRound(x0 - 1000*(-b));
                        pt2.y = cvRound(y0 - 1000*(a));
                        
                        //cv::rectangle( src, pt1, pt4, cv::Scalar(0, 0, 255));
                        cv::line( src, pt1, pt2, cv::Scalar(0,0,255), 1, cv::LINE_AA);
                    }
                }
            }
            
            cv::imshow("gray", dst);
            cv::waitKey(30);
            // Deal with edge cases later
            // if (roi.y > 0) {
            //     
            // }

            cv::rectangle(src, roi.tl(), roi.br(), cv::Scalar(0, 0, 255), 2, 8);
        }

    private:

        ros::NodeHandle nH;
        image_transport::ImageTransport it;

        cv::Mat frame;
        cv::Rect2d roi;
};


int main(int argc, char** argv) {
    

    std::string nodeName = "stemDetectionNode";
    ros::init(argc, argv, nodeName);
    
    stemDetector sD;

    // Window for GUI
    #ifdef GUI
        std::string windowName = "stemDetect";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    #endif

    // CV frame object
    cv::Mat frame = cv::imread("/home/kylejosling/cuke_ws/src/cuke_vision/include/test/89655140.jpg");
    // Get bounding box
    std::ifstream s("/home/kylejosling/cuke_ws/src/cuke_vision/include/test/89655140.txt");
    
    if (s.is_open()) {
        
        std::cout << "Stream is open " << std::endl;
        // Class, x, y, width, height
        // x and y are for middle of rectange
        float c, x, y, w, h;
        s>>c;
        s>>x;
        s>>y;
        s>>w;
        s>>h;
        std::cout << c << " " << x << " " << y << " " << w << " " << h << std::endl;
        
        // Get size
        cv::Size size = frame.size();
        std::cout << size.width << std::endl;
        std::cout << size.height << std::endl;
        std::cout << std::endl;
        int boxX = x*size.width - (w*size.width)/2 ;
        int boxY = y*size.height - (h*size.height)/2;
        int boxWidth = size.width*w;
        int boxHeight = size.height*h;
        std::cout << boxX << std::endl;
        std::cout << boxY << std::endl;
        std::cout << boxWidth << std::endl;
        std::cout << boxHeight << std::endl;

        cv::Rect2d boundingRect = cv::Rect2d( boxX, boxY, boxWidth, boxHeight);
        // cv::rectangle(frame, boundingRect.tl(), boundingRect.br(), cv::Scalar(0, 255, 0), 2, 8);
        
        sD.imageCallback(frame, boundingRect);

    }
    cv::imshow(windowName, frame); 
    cv::waitKey(30);

    ros::spin();

    #ifdef GUI
    cv::destroyWindow(windowName);
    #endif
    
    return 0;
}
