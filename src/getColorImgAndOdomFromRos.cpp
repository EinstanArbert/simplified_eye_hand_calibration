#include "getColorImgAndOdomFromRos.hpp"

CaptureData::CaptureData():out(std::string(getenv("HOME")) + "/.config/od_ros/calibData/pose.txt"){
}

void CaptureData::getImageColor_callback(const sensor_msgs::ImageConstPtr& msgRGB){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	colorImg = cv_ptrRGB->image;
	// cv_ptrRGB->header.stamp.toSec();  //时间戳

	// cv::cvtColor(colorImg, colorImg, CV_BGR2GRAY);
}

void CaptureData::saveOdomPose(){
	//!should add mutex lock
    out << basePose.x << "   " << basePose.y << "   " << basePose.z << "   "
    << basePose.qx << "   " << basePose.qy << "   " << basePose.qz << "   "
    << basePose.qw
    << std::endl;
}


void CaptureData::getBasePoseInOdom_callback(const nav_msgs::OdometryConstPtr& location_pose_ptr){
	nav_msgs::Odometry location_pose = *location_pose_ptr;
	if(!isnan(location_pose.pose.pose.position.x) && !isnan(location_pose.pose.pose.position.y))
	{
		basePose.x = location_pose.pose.pose.position.x;
		basePose.y = location_pose.pose.pose.position.y;
		basePose.z = location_pose.pose.pose.position.z;
		basePose.qx = location_pose.pose.pose.orientation.x;
		basePose.qy = location_pose.pose.pose.orientation.y;
		basePose.qz = location_pose.pose.pose.orientation.z;
		basePose.qw = location_pose.pose.pose.orientation.w;
	}
	else
	{
		basePose.x = 0.;
		basePose.y = 0.;
		basePose.z = 0.;
		basePose.qx = 0.;
		basePose.qy = 0.;
		basePose.qz = 0.;
		basePose.qw = 0.;
	}

}


void CaptureData::getBasePoseByLocation_callback(const geometry_msgs::PoseStampedConstPtr& location_pose_ptr){
	geometry_msgs::PoseStamped location_pose = *location_pose_ptr;
	if(!isnan(location_pose.pose.position.x) && !isnan(location_pose.pose.position.y))
	{
		basePose.x = location_pose.pose.position.x;
		basePose.y = location_pose.pose.position.y;
		basePose.z = location_pose.pose.position.z;
		basePose.qx = location_pose.pose.orientation.x;
		basePose.qy = location_pose.pose.orientation.y;
		basePose.qz = location_pose.pose.orientation.z;
		basePose.qw = location_pose.pose.orientation.w;
	}
	else
	{
		basePose.x = 0.;
		basePose.y = 0.;
		basePose.z = 0.;
		basePose.qx = 0.;
		basePose.qy = 0.;
		basePose.qz = 0.;
		basePose.qw = 0.;
	}

}