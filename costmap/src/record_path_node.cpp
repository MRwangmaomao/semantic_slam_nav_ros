/*
 * @Author: 王培荣
 * @Date: 2020-01-04 11:36:39
 * @LastEditTime : 2020-01-04 11:44:02
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/costmap/src/record_path_node.cpp
 */

#include <ros/ros.h>
#include <ros/console.h> 
#include <iostream>
#include <string> 
#include <fstream>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
using namespace std;
std::ofstream gtfile;

void write_gt_callback(const tf2_msgs::TFMessage::ConstPtr msg){
    // ROS_INFO_STREAM("y: " << mes->transforms.transform.translation.y);
    std::cout << (msg->transforms.at(0).header.stamp) << " " << msg->transforms.at(0).transform.translation.x << " " << msg->transforms.at(0).transform.translation.y << 
        " " << msg->transforms.at(0).transform.translation.z << " ";
    std::cout << msg->transforms.at(0).transform.rotation.w << " " << msg->transforms.at(0).transform.rotation.x << 
        " " << msg->transforms.at(0).transform.rotation.y << " " << msg->transforms.at(0).transform.rotation.z << std::endl;
    gtfile << (msg->transforms.at(0).header.stamp) << " " << msg->transforms.at(0).transform.translation.x << " " << msg->transforms.at(0).transform.translation.y << 
        " " << msg->transforms.at(0).transform.translation.z << " " << msg->transforms.at(0).transform.rotation.w
        << " " << msg->transforms.at(0).transform.rotation.x << " " << msg->transforms.at(0).transform.rotation.y 
        << " " << msg->transforms.at(0).transform.rotation.z << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "record_path");
    ros::start();
    ros::NodeHandle nh;
    std::cout << "\n-------------------------- start --------------------------\n" << std::endl;
    gtfile.open("/home/wpr/code/catkin_ws/src/orbslam_semantic_nav_ros/data/gt.txt",std::ios::in);
    if(!gtfile){
        std::cout << "Unable to open otfile";
        exit(1);
    } 
    ros::Subscriber sub_gt = nh.subscribe("/gt", 100, write_gt_callback);

    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    gtfile.close();
    return 0;
}

 
