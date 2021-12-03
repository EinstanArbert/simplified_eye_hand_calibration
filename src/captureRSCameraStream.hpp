/**
 * @file     captureRSCameraStream.hpp
 * @author zengbin
 * @brief    My Param doc
 * @version 0.1
 * @date 2020-11-19
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef captureRSCameraStream_hpp
#define captureRSCameraStream_hpp
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

#include <iostream>

using namespace cv;
using namespace std;

namespace pure_vo{
    struct Pose{
        float x, y, z, qx, qy, qz, qw;
    };
    enum direction
    {
        to_depth,
        to_color
    };
    enum VideoStreamFlag{
        depthStream,
        colorStream,
        bothStream,
        imuStream
    };
    struct IMUData
    {
        cv::Point3f accel;
        cv::Point3f gyro;
    };

    class CaptureRSCameraStream{
        public:
            /**
             * @brief    Construct a new Pure V O objectMy Param doc
             */
            CaptureRSCameraStream();
            /**
             * @brief    My Param doc
             * @param    flagMy Param doc
             */
            void captureVideoStream(VideoStreamFlag flag);

            void getBasePose_callback(const geometry_msgs::PoseStampedConstPtr& location_pose_ptr);

            cv::Mat depthImg, colorImg;
            IMUData imuData;
            std::ofstream out;

        private:
            rs2::pipeline pipe;
            rs2::config cfg;
            rs2::context ctx;
            rs2::frameset frameset;
            direction dir;
            int width_, height_, fps;

    };

}
#endif // !PURE_VO_H_