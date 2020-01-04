/*
 * @Author: your name
 * @Date: 2019-12-23 08:13:28
 * @LastEditTime : 2019-12-30 17:17:01
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/costmap_lrgbd_ros/include/costmap_lrgbd_ros/dwa_planning.h
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue> 
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class DWAPlanning{

public:
    DWAPlanning();
    ~DWAPlanning(void); 
    void init(std::string config_file_path, int depthscale, Eigen::Matrix3d camera_k, double map_width, double map_height, double resolution_size); 
    bool move(long int robot_pose_id, Eigen::Matrix4d robot_pose, const cv::Mat& config_map, double & go_v, double & turn_v);
    visualization_msgs::Marker dest_waypoint_pub();
    
private:
    void getWayPoint();
    void getRobotLocalization();
    void setWayPointInCostmap();
    void getAllWayPoints();
    bool isArriveWayPoint();
    bool isArriveDestination();
    bool dwa_control(const cv::Mat& config_map);
    void dwa_display(cv::Mat& config_map);
    void drawArrow(cv::Mat& img, int start_x, int start_y, double theta, int arraw_length);
    Eigen::VectorXd motion(Eigen::VectorXd &x, double v_head, double v_theta);
    void calc_to_waypoint_cost();
    double max_acc_x_;
    double max_acc_theta_;
    double max_yaw_rate_;
    double max_speed_;
    double min_speed_;
    double foresee_;
    double sim_time_;
    double region_foresee_;
    double short_foresee_rate_;
    double sim_period_;
    bool aggressive_mode_;
    double go_v_;
    double turn_v_;
    double distance_threshold_;
    double depthScale_;
    double map_width_;
    double map_height_;
    double resolution_size_;
    double delta_yam_rate_;
    double delta_speed_;
    int waypoint_id_;
    long int robot_pose_id_;
    bool first_flag_;
    Eigen::Matrix3d camera_k_;
    std::string waypoints_file_path_;
    std::queue<std::vector<double>> waypoints_queue_;
    Eigen::Matrix4d T_robot_waypoint_;
    std::vector<double> robot_waypoint_; // 机器人路标点
    Eigen::Matrix4d robot_pose_; // 机器人在世界中的位姿
    Eigen::Vector3d robot_wapoint_in_costmap_; //机器人在代价地图中的位置

};
