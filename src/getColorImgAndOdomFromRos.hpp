/**
 * @file     getColorImgAndOdomFromRos.hpp
 * @author zengbin
 * @brief
 * @version 0.1
 * @date 2022-05-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef getColorImgAndOdomFromRos_hpp
#define getColorImgAndOdomFromRos_hpp

#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>


struct Pose{
    float x, y, z, qx, qy, qz, qw;
};

class CaptureData{
        public:
            /**
             * @brief    Construct a new Pure V O objectMy Param doc
             */
            CaptureData();

            // 获取color image
            void getImageColor_callback(const sensor_msgs::ImageConstPtr& msgRGB);

            // 获取agv在odom坐标系下的位姿
            void getBasePoseInOdom_callback(const nav_msgs::OdometryConstPtr& location_pose_ptr);

            cv::Mat colorImg;
            // Pose basePose;
            std::ofstream out;
};

#endif