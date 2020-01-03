/*
 * @Author: 王培荣
 * @Date: 2020-01-03 15:30:25
 * @LastEditTime : 2020-01-04 00:55:14
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/aip-cpp/src/face_pub_node.cpp
 */ 
// 参考 https://ai.baidu.com/ai-doc/FACE/Uk37c1r11
#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <stdlib.h>
#include <iostream> 
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h> 
 
#include "base/base.h"
#include "face.h"

ros::Subscriber sub_img;
aip::Face client("18166213", "HVV12BIfUj6IEegBfC9KrRZK", "azRZb0yDtaCdVtG1BF6zmYADAV3nRytX");
std::string rospackage_path;


void img_callback(const sensor_msgs::ImageConstPtr &msgRGB)
{   
    try
    { 
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8); 
        Json::Value result;
        std::string image;
        std::string path = rospackage_path + "data/face.jpg";
        cv::imwrite(path.c_str(), cv_image->image);
        aip::get_file_content(path.c_str(), &image); 
        std::string image_type = "BASE64"; 
        std::string group_id_list = "904"; 
        // 调用人脸搜索
        result = client.multi_search(image, image_type, group_id_list, aip::null);
        
        std::cout << result << std::endl <<
        "group_id:  " << result["result"]["user_list"][0]["group_id"] << std::endl << 
        "user_id:  " << result["result"]["user_list"][0]["user_id"] << std::endl <<
        "score" << result["result"]["score"][0]["score"] << 
        std::endl << std::endl << std::endl;  
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


int main(int argc, char ** argv){
    ros::init(argc,argv,"face_pub");
    ros::NodeHandle nh;
    ros::start();
    if(argc != 2)
    {
        std::cerr << std::endl << "缺少参数" << std::endl << 
        "Usage: rosrun slam_semantic_nav_ros face_pub_node" << std::endl;
        return 1;
    }
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
        std::cerr << std::endl <<
            std::endl << 
            "---------------------------------------------" << std::endl << 
            "---------------------------------------------" << std::endl << 
            "您的文件路径设置错误了，请在roslaunch中修改配置文件的路径！！！" << std::endl <<
            std::endl <<
            std::endl <<
            "祝您实验取得成功。" << std::endl << 
            "---------------------------------------------" << std::endl << 
            "---------------------------------------------" << std::endl;
            exit(1);
    }
    fsSettings["rospackage_path"] >> rospackage_path;

    sub_img = nh.subscribe("/usb_cam/image_raw", 1, img_callback);
    ros::spin(); 
    ros::shutdown(); 
    return 0;
}

