/*
 * @Author: 王培荣
 * @Date: 2019-12-29 11:18:52
 * @LastEditTime : 2019-12-31 15:56:37
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/include/Node.h
 */ 

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <orb_slam2_ros/dynamic_reconfigureConfig.h>

// #include "orb_slam2_ros/SaveMap.h" // srv中的SaveMap自动生成的节点

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <System.h> // ORBSLAM 头文件
#include "orbslam_semantic_nav_ros/Arrived.h" 
// #include "costmap_lrgbd_ros/Arrived.h"



class Node
{
  public:
    Node (ros::NodeHandle &node_handle, std::string config_file_path);
    ~Node ();

  protected:
    bool Update ();
    void init_slam();
    ORB_SLAM2::System* orb_slam_; // 定义ORBSLAM系统

    ros::Time current_frame_time_;
    std::string color_img_topic_;
    std::string depth_img_topic_;

  private:
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    // void isArrivedCallback(const costmap_lrgbd_ros::Arrived::ConstPtr arrived_msg);
    void isArrivedCallback(const orbslam_semantic_nav_ros::Arrived::ConstPtr arrived_msg);
    // void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
    // bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);

    tf::Transform TransformFromMat (cv::Mat position_mat);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    // dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Subscriber is_arrived_sub_;
    ros::ServiceServer service_server_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    std::string rospackage_path_;
    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    std::string settings_file_name_param_;
    std::string folder_path_;
    bool close_system_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif //ORBSLAM2_ROS_NODE_H_

