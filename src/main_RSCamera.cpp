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


int main(){
    CaptureRSCameraStream captureImage;
	cv::Mat colorImg;

    int imgID = 0;
    int key;

    while(true){
        captureImage.captureVideoStream(VideoStreamFlag::colorStream);
        colorImg = captureImage.colorImg;
        cv::imshow("colorImage", colorImg);
        key = cvWaitKey(2);
        if(key != -1){
            if((char)key == 's'){
                std::stringstream ss1;
                ss1 << "./images/"<< imgID << ".png";
                cv::imwrite(ss1.str().c_str(), colorImg);
                imgID += 1;
            }
        }
    }

	return 0;
}