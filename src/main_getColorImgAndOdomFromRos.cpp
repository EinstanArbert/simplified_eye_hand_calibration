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

    ros::init(argc, argv, "getColorImgAndOdomFromRos");
    ros::NodeHandle n("~");
    bool manualMode;
    n.param("manualMode", manualMode, false);
	CaptureData captureData;
	cv::Mat colorImg;

    int imgID = 0;
    int key;

    ros::Subscriber color_sub = n.subscribe("/camera/rgb/image_raw", 1, &CaptureData::getImageColor_callback, &captureData);
    // ros::Subscriber odom_sub = n.subscribe("/odom", 1, &CaptureData::getBasePoseInOdom_callback, &captureData);
    ros::Subscriber odom_sub = n.subscribe("/location", 1, &CaptureData::getBasePoseByLocation_callback, &captureData);
    ros::Rate loop_rate(1);
    ros::Rate loop_rate2(0.5);
    while(ros::ok()){
        colorImg = captureData.colorImg;
        if(colorImg.empty()){
            std::cout << "Waiting image ..." << std::endl;
            ros::spinOnce();
            loop_rate2.sleep();
            continue;
        }

        //手动点击左键在打开的图片窗口，然后按s键保存图片与odom位置到指定文件
        if(manualMode){
            cv::imshow("colorImage", colorImg);
            // key = cvWaitKey(2);
            key = cv::waitKey(2);
            if(key != -1){
                if((char)key == 's'){
                    captureData.saveOdomPose();
                    std::stringstream ss1;
                    ss1 << calibDataDir << imgID << ".png";
                    cv::imwrite(ss1.str().c_str(), colorImg);
                    imgID += 1;
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
        }
        else{
            //自动按loop_rate的频率保存图片与odom的值到指定文件
            captureData.saveOdomPose();
            std::stringstream ss1;
            ss1 << calibDataDir << imgID << ".png";
            cv::imwrite(ss1.str().c_str(), colorImg);
            imgID += 1;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

	return 0;
}