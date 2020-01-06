/*
 * @Author: 王培荣
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-01-06 00:16:12
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/src/RGBDNode.cpp
 */
#include "RGBDNode.h"


RGBDNode::RGBDNode (ros::NodeHandle &node_handle, std::string config_file_path) : Node (node_handle, config_file_path) {
  std::cout << "Subscriber color_img_topic: " << color_img_topic_ << "." << std::endl;
  std::cout << "Subscriber depth_img_topic: " << depth_img_topic_ << "." << std::endl; 
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, color_img_topic_, 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, depth_img_topic_, 1);

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
  std::cout << "Start to subscribe topic. " << std::endl;
  init_slam();
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat. 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  if(Update()){
    std::cout << "关闭ROS系统!" << std::endl;
 
  }
}
