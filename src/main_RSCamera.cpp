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

#include "captureRSCameraStream.hpp"
#include <thread>

using namespace cv;
using namespace std;
using namespace pure_vo;


int main(int argc, char **argv){
    ros::init(argc, argv, "calibLidarToBase");
    ros::NodeHandle n;

    CaptureRSCameraStream captureImage;
	cv::Mat colorImg;

    int imgID = 0;
    int key;

    ros::Subscriber location_pose_sub = n.subscribe("scan", 1, &CaptureRSCameraStream::getBasePose_callback, &captureImage);
    // ros::Rate loop_rate(20);
    while(ros::ok()){
        captureImage.captureVideoStream(VideoStreamFlag::colorStream);
        colorImg = captureImage.colorImg;
        cv::imshow("colorImage", colorImg);
        key = cvWaitKey(2);
        if(key != -1){
            if((char)key == 's'){
                std::stringstream ss1;
                ss1 << "./data/"<< imgID << ".png";
                cv::imwrite(ss1.str().c_str(), colorImg);
                ros::spinOnce();
                imgID += 1;
                // loop_rate.sleep();
            }
        }
    }

	return 0;
}