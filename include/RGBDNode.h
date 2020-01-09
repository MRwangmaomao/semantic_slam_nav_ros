/*
 * @Author: 王培荣
 * @Date: 2019-12-29 11:15:10
 * @LastEditTime : 2020-01-08 22:19:14
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/include/RGBDNode.h
 */ 

#ifndef ORBSLAM2_ROS_RGBDODE_H_
#define ORBSLAM2_ROS_RGBDODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf/transform_broadcaster.h>

#include <System.h>
#include <Node.h>


class RGBDNode : public Node
{
  public:
    RGBDNode (ros::NodeHandle &node_handle, std::string config_file_path);
    ~RGBDNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    
  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
};

#endif //ORBSLAM2_ROS_RGBDODE_H_
