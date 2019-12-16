#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

#include <algorithm>
#include <string>
#include <iostream>
#include <cmath>

// ** BOUNDING BOX TEST NODE ** //
int main(int argc, char** argv) {

    rs2::pipeline pp;
    rs2::config cfg;
    float ResultVector[3];
    float InputPixelAsFloat[2];
    InputPixelAsFloat[0] = 431;
    InputPixelAsFloat[1] = 509;

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16 , 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    auto MyPipelineProfile = pp.start(cfg); 

    rs400::advanced_mode MyDevice = MyPipelineProfile.get_device();
    rs2::depth_sensor DepthSensor = MyDevice.first<rs2::depth_sensor>();
    float DepthUnits = DepthSensor.get_option(RS2_OPTION_DEPTH_UNITS);

    auto DepthStream = MyPipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

    rs2_intrinsics DepthIntrinsics = DepthStream.get_intrinsics();


     for (int i = 0; i<10; i++)//to skip first few frames when device just initiated
        auto frames = pp.wait_for_frames();

    auto frames = pp.wait_for_frames();

    rs2::depth_frame depthFrame = frames.get_depth_frame();
    float distance = depthFrame.get_distance(431, 509);

    rs2_deproject_pixel_to_point(ResultVector, &DepthIntrinsics, InputPixelAsFloat, distance);

     std::cout << "x = " << ResultVector[0] << ", y = " << ResultVector[1] << ", z = " << ResultVector[2] << std::endl;

    pp.stop();

    // Initialize point publisher
    ros::init(argc, argv, "points_pub");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(30);

    visualization_msgs::Marker points;
    points.header.frame_id = "/camera_depth_optical_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "point";
    points.pose.orientation.w = 1.0;

    points.id = 0;
    
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

    points.color.g = 1.0f;
    points.color.a = 1.0;
    
    geometry_msgs::Point p;
    p.x = ResultVector[0];
    p.y = ResultVector[1];
    p.z = ResultVector[2];

    points.points.push_back(p);

    while (ros::ok()) {
        marker_pub.publish(points);
    }

    return 0;
}
