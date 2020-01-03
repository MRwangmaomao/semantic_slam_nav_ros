/*
 * @Author: 王培荣
 * @Date: 2019-12-29 01:47:26
 * @LastEditTime : 2020-01-04 00:33:39
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav/src/kinect_camera_node.cpp
 */

#include<iostream> 
#include <ros/ros.h> 
#include "RGBDNode.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;
    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: ./kinect_camera_node path_to_settings" << std::endl;
        return 1;
    }
    ROS_INFO("------------------------start------------------------------"); 
    RGBDNode kinect_node(nh, argv[1]);
    ros::spin();
    ros::shutdown();
    ROS_INFO("------------------------end------------------------------");
    return 0;
}

