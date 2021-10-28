/**
 * @file     captureRSCameraStream.cpp
 * @author zengbin
 * @brief    My Param doc
 * @version 0.1
 * @date 2020-11-19
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "captureRSCameraStream.hpp"

namespace pure_vo{
/**
 * @brief    Construct a new Capture R S Camera Stream:: Capture R S Camera Stream object
 * @param    filename
 * @date 2021-08-06
 */
CaptureRSCameraStream::CaptureRSCameraStream():dir(to_depth){
	width_ = 640;
	height_ = 480;
	fps = 60;


	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
	cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16,fps);
	cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    auto profile = pipe.start(cfg);
    auto sensor = profile.get_device().first<rs2::color_sensor>();
    // sensor.set_option(RS2_OPTION_EXPOSURE, EXPOSURE_VALUE);  //set EXPOSURE_VALUE
    // sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);  //set AUTO_EXPOSURE
    // sensor.set_option(RS2_OPTION_GAIN, GAIN_VALUE);  //set GAIN
}


/**
 * @brief    capture Video Stream
 * @param    flag
 * @date 2021-08-06
 */
void CaptureRSCameraStream::captureVideoStream(VideoStreamFlag flag){
    frameset = pipe.wait_for_frames();

	switch (flag){
		case VideoStreamFlag::depthStream:{
			auto depthFrames = frameset.get_depth_frame();
			depthImg = Mat(Size(width_, height_), CV_16UC1, (void*)depthFrames.get_data(), Mat::AUTO_STEP);
			// std::cout << "Captured depth stream only.";
			break;
		}
		case VideoStreamFlag::colorStream:{
			auto colorFrames = frameset.get_color_frame();
			colorImg = Mat(Size(width_, height_), CV_8UC3, (void*)colorFrames.get_data(), Mat::AUTO_STEP);
			// cv::cvtColor(colorImg,colorImg,CV_BGR2RGB);
			// std::cout << "Captured color stream only.";
			break;
		}
		case VideoStreamFlag::imuStream:{
			// Get imu data
			if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
			{
				rs2_vector accel_sample = accel_frame.get_motion_data();
				imuData.accel.x = accel_sample.x;
				imuData.accel.y = accel_sample.y;
				imuData.accel.z = accel_sample.z;
				std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z;
			}
			if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
			{
				rs2_vector gyro_sample = gyro_frame.get_motion_data();
				imuData.gyro.x = gyro_sample.x;
				imuData.gyro.y = gyro_sample.y;
				imuData.gyro.z = gyro_sample.z;
				std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z;
			}
			std::cout << "Captured IMU data.";
			break;
		}
		case VideoStreamFlag::bothStream:{
			if (dir == to_depth)
			{
				// Align all frames to depth viewport
				rs2::align align_to_depth(RS2_STREAM_DEPTH);
				frameset = align_to_depth.process(frameset);
			}
			else
			{
				// Align all frames to color viewport
				rs2::align align_to_color(RS2_STREAM_COLOR);
				frameset = align_to_color.process(frameset);
			}
			auto depthFrames = frameset.get_depth_frame();
			depthImg = Mat(Size(width_, height_), CV_16UC1, (void*)depthFrames.get_data(), Mat::AUTO_STEP);
			auto colorFrames = frameset.get_color_frame();
			colorImg = Mat(Size(width_, height_), CV_8UC3, (void*)colorFrames.get_data(), Mat::AUTO_STEP);
			// cv::cvtColor(colorImg,colorImg,CV_BGR2RGB);
			std::cout << "Captured color and depth stream.";
			break;
		}
		default:
			break;
	}
}

}