#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/tracking/tracking.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>

#include "cuke_vision/detectObject.hpp"
#include "cuke_vision/boundingBoxMsg.h"

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& out);

class objectDetector {

    public:

        // Initialization list
        objectDetector():

        // Initialize image transport instance
        it(nH),
        colorImageSub(it, colorImageTopic, 1),
        depthImageSub(it, depthImageTopic, 1),
        sync(SyncPolicy(10), colorImageSub, depthImageSub)
        { 
            
            // Bind subscriber callbacks using boost
            sync.registerCallback(boost::bind( &objectDetector::imageCallback, this, _1, _2));

            // Initialize publisher 
            boundingBoxPub = nH.advertise<cuke_vision::boundingBoxMsg>(boxTopic, 1000);

            // Load classes
            std::string line;
            std::ifstream ifs(classesFile.c_str());
            while (getline(ifs, line)) classes.push_back(line);

            // Load the network
            net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

            // Default backend should be OpenCV
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

            #ifdef GUI
            namedWindow(kWinName, cv::WINDOW_NORMAL);
            #endif

            // Load camera parameters

        }

        ~objectDetector() {
            
            #ifdef GUI
            cv::destroyWindow(kWinName);
            #endif
        }

        void imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg);
        
    private:
        
        // Sync policy for synchronization
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

        const std::string colorImageTopic = "/camera/color/image_raw";
        const std::string depthImageTopic = "/camera/aligned_depth_to_color/image_raw";
        const std::string boxTopic = "/2d_bounding_boxes";

        ros::NodeHandle nH;
        image_transport::ImageTransport it;

        // Image transport subscriber
        image_transport::SubscriberFilter colorImageSub;
        image_transport::SubscriberFilter depthImageSub;

        // Define synchronizer for callbacks
        message_filters::Synchronizer<SyncPolicy> sync;

        // Publisher
        ros::Publisher boundingBoxPub;

        // Window name
        const std::string kWinName = "Object Detection";

        // Class parameters
        std::vector<std::string> classes;
        std::string classesFile = "/home/kylejosling/Downloads/cuke-training/classes.names";

        // Model configuration files
        cv::String modelConfiguration = "/home/kylejosling/Downloads/cuke-training/darknet-yolov3.cfg";
        cv::String modelWeights = "/home/kylejosling/Downloads/cuke-training/weights/darknet-yolov3_final.weights";

        // Neural network
        cv::dnn::Net net;

        // Network parameters
        int inpWidth = 416;
        int inpHeight = 416;

        float confThreshold = 0.5; // Confidence threshold
        float nmsThreshold = 0.4;  // Non-maximum suppression threshold

        cv::Mat frame;
        cv::Mat blob;

        // Draw the predicted bounding box
        void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

        // Get the names of the output layers
        std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);

        // Remove the bounding boxes with low confidence using non-maxima suppression
        void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs);
};


void objectDetector::imageCallback(const sensor_msgs::ImageConstPtr &colorImageMsg, const sensor_msgs::ImageConstPtr &depthImageMsg) {

    try {
        
        ROS_INFO("Frame received");

        frame = cv_bridge::toCvCopy(colorImageMsg, "bgr8")->image;

        // Create a 4D blob from a frame.
        cv::dnn::blobFromImage(frame, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        std::vector<cv::Mat> outs;
        // TODO is this right? old function was causing linker errors 
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence, publish message
        postprocess(frame, outs);

        // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        std::vector<double> layersTimes;
        double freq = cv::getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        std::string label = cv::format("Inference time for a frame : %.2f ms", t);

        #ifdef GUI
        cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        cv::imshow(kWinName, frame);
        cv::waitKey(30);
        #endif

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Image encoding error %s", e.what());
    }
}

void objectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) {

    //Draw a rectangle displaying the bounding box
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    
    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    } else {
        std::cout << "Classes r empty" << std::endl;
    }
    
    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);
}


std::vector<cv::String> objectDetector::getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

void objectDetector::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs) {

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Message to publish
    cuke_vision::boundingBoxMsg msg;
    
    for (size_t i = 0; i < outs.size(); ++i) {

        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {

            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i) {

        int idx = indices[i];
        cv::Rect box = boxes[idx];
        #ifdef GUI
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
        #endif
         
        ROS_INFO("Center of cucumber located at : ");

        // Pack the message
        msg.x.push_back(box.x);
        msg.y.push_back(box.y);
        msg.height.push_back(box.height);
        msg.width.push_back(box.width);

    }

    // Publish the message
    boundingBoxPub.publish(msg);
}


int main(int argc, char** argv) {

    std::cout << "CV_major : " << CV_MAJOR_VERSION << std::endl;
    std::cout << "CV_minor : " << CV_MINOR_VERSION << std::endl;

    std::string nodeName = "cukeVisionNode";
    ros::init(argc, argv, nodeName);

    objectDetector oD;

    ros::spin();

    return 0;
}


