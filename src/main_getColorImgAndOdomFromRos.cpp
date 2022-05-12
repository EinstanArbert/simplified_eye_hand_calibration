/**
 * @file     main.cpp
 * @author zengbin
 * @brief    My Param doc
 * @version 0.1
 * @date 2020-07-14
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <thread>
#include "getColorImgAndOdomFromRos.hpp"

using namespace cv;
using namespace std;



int main(int argc, char **argv){
    std::string calibDataDir = std::string(getenv("HOME")) + "/.config/od_ros/calibData/";
    // std::ofstream out(calibDataDir + "pose.txt");

    ros::init(argc, argv, "calibCameraToOdom");
    ros::NodeHandle n;
	CaptureData captureData;
	cv::Mat colorImg;

    int imgID = 0;
    int key;

    ros::Subscriber color_sub = n.subscribe("/camera/rgb/image_raw", 1, &CaptureData::getImageColor_callback, &captureData);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, &CaptureData::getBasePoseInOdom_callback, &captureData);
    // ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();

        colorImg = captureData.colorImg;
        if(colorImg.empty()){
            std::cout << "Waiting image ..." << std::endl;
            continue;
        }
        cv::imshow("colorImage", colorImg);

        key = cvWaitKey(2);
        if(key != -1){
            if((char)key == 's'){
                std::stringstream ss1;
                ss1 << calibDataDir << imgID << ".png";
                cv::imwrite(ss1.str().c_str(), colorImg);
                // ros::spinOnce();
                imgID += 1;
                // loop_rate.sleep();
            }
        }
    }

	return 0;
}