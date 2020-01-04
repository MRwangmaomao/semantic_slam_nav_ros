
#include "costmap_lrgbd_ros/dwa_planning.h"

DWAPlanning::DWAPlanning(){

}


DWAPlanning::~DWAPlanning(void){

}

void DWAPlanning::init(std::string config_file_path, int depthscale, Eigen::Matrix3d camera_k, double map_width, double map_height, double resolution_size){
    depthScale_ = depthscale;
    camera_k_ = camera_k;
    map_width_ =map_width;
    map_height_ =map_height;
    resolution_size_ = resolution_size;
    waypoint_id_ = 0;
    robot_pose_id_ = 0;
    first_flag_ = true;
    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
    max_acc_x_ = fsSettings["max_acc_x"];
    max_speed_ = fsSettings["max_speed"];
    min_speed_ = fsSettings["min_speed"];
    foresee_ = fsSettings["foresee"]; 
    sim_time_ = fsSettings["sim_time"];
    region_foresee_ = fsSettings["region_foresee"];
    short_foresee_rate_ = fsSettings["short_foresee_rate"];
    distance_threshold_ = fsSettings["distance_threshold"];
    sim_period_ = fsSettings["sim_period"];
    max_yaw_rate_ = fsSettings["max_yaw_rate"];
    delta_yam_rate_ = fsSettings["delta_yam_rate"];
    delta_speed_ = fsSettings["delta_speed"];
    fsSettings["waypoints_file"] >> waypoints_file_path_; 
    std::ifstream waypoint_file(waypoints_file_path_);
    std::cout << "\nWaypoint Lists:\n";
    std::string temp;
    double waypoint_x, waypoint_y;
    double q_x, q_y, q_z, q_w;
    std::string temp_w;
    while(getline(waypoint_file,temp)) //按行读取字符串 
	{
        
        std::stringstream input(temp);
        input>>temp_w; // time
        input>>temp_w; 
        waypoint_x = atof(temp_w.c_str());
        input>>temp_w; 
        waypoint_y = atof(temp_w.c_str());
        input>>temp_w;
        input>>temp_w;
        q_w = atof(temp_w.c_str());
        input>>temp_w;
        q_x = atof(temp_w.c_str());
        input>>temp_w;
        q_y = atof(temp_w.c_str());
        input>>temp_w;
        q_z = atof(temp_w.c_str());
         
		std::cout << "  " << waypoint_x << " " << waypoint_y << std::endl;
        std::vector<double> point_temp;
        point_temp.push_back(waypoint_x);
        point_temp.push_back(waypoint_y);
        point_temp.push_back(0);
        point_temp.push_back(q_w);
        point_temp.push_back(q_x);
        point_temp.push_back(q_y);
        point_temp.push_back(q_z);
        waypoints_queue_.push(point_temp);
	} 
	waypoint_file.close();
    getWayPoint();

    std::cout << "\nConfigration the DWA parameters: " << std::endl << 
        "   max_acc_x: " << max_acc_x_ << std::endl << 
        "   max_speed: " << max_speed_ << std::endl << 
        "   min_speed: " << foresee_ << std::endl << 
        "   foresee: " << max_acc_x_ << std::endl << 
        "   sim_time: " << sim_time_ << std::endl << 
        "   region_foresee: " << region_foresee_ << std::endl << 
        "   short_foresee_rate: " << short_foresee_rate_ << std::endl << 
        "   sim_period: " << sim_period_ << std::endl << 
        "   waypoints_file_path: " << waypoints_file_path_ << std::endl
        << std::endl;
}

void DWAPlanning::getWayPoint(){
    if(waypoints_queue_.size() > 0){
        robot_waypoint_ = waypoints_queue_.front();
        waypoints_queue_.pop();
        waypoint_id_++;
        std::cout << "\n\n ------------------------------\n The " << waypoint_id_ << " points: " 
        << robot_waypoint_[0] << " " << robot_waypoint_[1] << std::endl;
    }
    waypoints_queue_.size();
}

void DWAPlanning::setWayPointInCostmap(){
    
    Eigen::Matrix4d T_world_waypoint;
    T_world_waypoint  << 0, 0, 0, 0, 
                         0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 1;
 
    // 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)  
    T_world_waypoint(0,3) = robot_waypoint_[0];
    T_world_waypoint(1,3) = robot_waypoint_[1];
    T_world_waypoint(2,3) = robot_waypoint_[2];
    Eigen::Quaterniond q_world_waypoint;
    q_world_waypoint.w() = robot_waypoint_[3];
    q_world_waypoint.x() = robot_waypoint_[4];
    q_world_waypoint.y() = robot_waypoint_[5];
    q_world_waypoint.z() = robot_waypoint_[6];
    Eigen::Matrix3d r_world_waypoint;
    r_world_waypoint = q_world_waypoint.matrix();
    T_world_waypoint.block(0,0,3,3) = r_world_waypoint;
    Eigen::Vector4d t_world_waypoint(robot_waypoint_[0], robot_waypoint_[1], robot_waypoint_[2], 1);  
    // 求出路标点在机器人坐标系下的坐标
    T_robot_waypoint_ = robot_pose_.inverse() * T_world_waypoint;  
    // std::cout << "\nActrual position is: " << robot_wapoint_in_costmap_(0) << "     " << robot_wapoint_in_costmap_(1) << std::endl;
    // ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw, 0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Matrix3d r_robot_waypoint;
    r_robot_waypoint = T_robot_waypoint_.block(0,0,3,3);
    Eigen::Vector3d euler_angles = r_robot_waypoint.eulerAngles(2, 1, 0); 
    
    // 在代价地图中可视化的路标点坐标
    robot_wapoint_in_costmap_(0) = T_robot_waypoint_(0,3)/resolution_size_ + (map_height_/resolution_size_)/2;
    robot_wapoint_in_costmap_(1) = -T_robot_waypoint_(1,3)/resolution_size_ + (map_width_/resolution_size_)/2;
    robot_wapoint_in_costmap_(2) = euler_angles(0);
    // std::cout << "\nrobot in image map: " << robot_wapoint_in_costmap_(0) << "      " << robot_wapoint_in_costmap_(1)  << std::endl;
}

visualization_msgs::Marker DWAPlanning::dest_waypoint_pub()
{
    visualization_msgs::Marker dest_mark;
    dest_mark.header.frame_id = "/map";
    dest_mark.header.stamp = ros::Time::now(); 
    dest_mark.type = visualization_msgs::Marker::CUBE;
    dest_mark.id = 0;
    dest_mark.ns = "basic_shapes";
    dest_mark.scale.x = 0.1;
    dest_mark.scale.y = 0.1;
    dest_mark.scale.z = 0.1;
    dest_mark.color.b = 0.0;
    dest_mark.color.g = 0.0;
    dest_mark.color.r = 1.0;
    dest_mark.color.a = 1.0;
    dest_mark.lifetime = ros::Duration(); 
    geometry_msgs::Pose pose; 
    pose.position.x = robot_waypoint_[0];
    pose.position.y = robot_waypoint_[1];
    pose.position.z = 0;
    pose.orientation.w = 1.0;
    dest_mark.pose = pose; 
    return dest_mark;
}

// 根据距离误差，后面还需要添加角度误差
bool DWAPlanning::isArriveWayPoint(){ 
    double distance_err = sqrt((robot_pose_(0,3) - robot_waypoint_[0])*(robot_pose_(0,3) - robot_waypoint_[0]) +
    (robot_pose_(1,3) - robot_waypoint_[1])*(robot_pose_(1,3) - robot_waypoint_[1])); 
    if(distance_err < distance_threshold_){      
        return true;
    }
    return false;
}

bool DWAPlanning::isArriveDestination(){
    if(waypoints_queue_.size() == 0 && isArriveWayPoint())
    {
        return true;
    } 
    return false;
}

void DWAPlanning::dwa_display(cv::Mat& dwa_image_map){

    int cross_line_size = 10;
    cv::line(dwa_image_map, cvPoint(robot_wapoint_in_costmap_(0)-cross_line_size/2,robot_wapoint_in_costmap_(1)), cvPoint(robot_wapoint_in_costmap_(0)+cross_line_size/2, robot_wapoint_in_costmap_(1)), cv::Scalar(0,0,255), 1, 8, 0); 
	cv::line(dwa_image_map, cvPoint(robot_wapoint_in_costmap_(0),robot_wapoint_in_costmap_(1)-cross_line_size/2), cvPoint(robot_wapoint_in_costmap_(0), robot_wapoint_in_costmap_(1)+cross_line_size/2), cv::Scalar(0,0,255), 1, 8, 0);

    cv::namedWindow("dwa");
    cv::imshow("dwa",dwa_image_map);
    cvWaitKey(1);
}

void DWAPlanning::drawArrow(cv::Mat& img, int start_x, int start_y, double theta, int arraw_length)
{ 
    int arraw_x = start_x + arraw_length * cos(theta);
    int arraw_y = start_y - arraw_length * sin(theta);
    cv::line(img, cv::Point(start_x, start_y), 
    cv::Point(arraw_x, arraw_y), cv::Scalar(255, 0, 255), 1, 8);
}

Eigen::VectorXd DWAPlanning::motion(Eigen::VectorXd &x, double v_head, double v_theta){
    x(0) = v_head * sim_time_;
    x(1) = 0;
    x(2) = v_theta * sim_time_;
    x(3) = v_head;
    x(4) = v_theta;
    return x;
}


void DWAPlanning::calc_to_waypoint_cost(){
    double min_cost_v = 0;
    double min_cost_theta = 0;
    double min_distant_cost = 99;
    double min_angular_cost = 99;
    double distant_cost;
    double angular_cost; 
    for(double speed = min_speed_; speed < max_speed_; speed += delta_speed_){
        distant_cost = sqrt((speed * sim_time_ - T_robot_waypoint_(0,3)) * ((speed * sim_time_ - T_robot_waypoint_(0,3))));
        if(distant_cost < min_distant_cost){
            min_distant_cost = distant_cost;
            go_v_ = speed;
        }
    } 
    for(double theta_v = -max_yaw_rate_; theta_v < max_yaw_rate_; theta_v += delta_yam_rate_){  
        angular_cost = sqrt((robot_wapoint_in_costmap_(2) - theta_v * sim_time_)*(robot_wapoint_in_costmap_(2) - theta_v * sim_time_));
        if(angular_cost < min_angular_cost){
            min_angular_cost = angular_cost;
            turn_v_ = theta_v;
        }
    }
}

/**
 * 1 先检测当前目标点是否位于可通行区域内
 *      1.1 如果在通行区域内计算生成速度和转角指令
 *      1.2 如果目标点在未知区域中，目标点在深度相机视角范围外，控制机器人旋转；在视角范围内，发送停止命令
 *      1.3 如果目标点在障碍物区域中，则发送停止命令
 * 
 * 2 在通行区域内计算速度和转角
 *      2.1 动态窗口法生成一些预选的速度和角速度值
 *      2.2 根据运动模型生成轨迹，判断哪条轨迹可以最快靠近目标位置
 */
bool DWAPlanning::dwa_control(const cv::Mat& config_map){
    cv::Mat dwa_image_map = config_map.clone(); 
    uint x = robot_wapoint_in_costmap_(0);
    uint y = robot_wapoint_in_costmap_(1); 
    // std::cout << "vec3b" << dwa_image_map.at<cv::Vec3b>(y,x) << std::endl;
    cv::Vec3b color_point = dwa_image_map.at<cv::Vec3b>(y,x);
    bool if_dwa_succ = true; 
    
    // 在可通行区域内部
    if(color_point(0) == 255 && color_point(1) == 192 && color_point(2) == 203){  
        calc_to_waypoint_cost();
    }
 
    // 在未知区域
    if(color_point(0) == 0 && color_point(1) == 0 && color_point(2) == 0){  
        std::cout << "I don't know where! " << std::endl;
        if_dwa_succ = false;
    }
    
    // 在障碍物区域
    if((color_point(0) == 0 && color_point(1) == 255 && color_point(2) == 0) || 
    (color_point.val[0] == 255 && color_point.val[1] == 0 && color_point.val[2] == 0)){
        std::cout << "There is obstacal!" << std::endl;
        if_dwa_succ = false; 
    }
    
    drawArrow(dwa_image_map, (map_width_/resolution_size_)/2, (map_height_/resolution_size_)/2, 0.3, (int)(40 * 0.4/max_speed_));
    drawArrow(dwa_image_map, robot_wapoint_in_costmap_(0), robot_wapoint_in_costmap_(1), robot_wapoint_in_costmap_(2), 10);
    dwa_display(dwa_image_map);
    return if_dwa_succ;
}

bool DWAPlanning::move(long int robot_pose_id, Eigen::Matrix4d robot_pose, const cv::Mat& config_map, double & go_v, double & turn_v){
    robot_pose_id_ = robot_pose_id;
    robot_pose_ = robot_pose;
    
    // 更新代价地图中路标点的坐标
    setWayPointInCostmap();


    if(isArriveDestination() == true){
        go_v = 0.0;
        turn_v = 0.0;
        std::cout << "\n\nI have arrived !!!\n\n" << std::endl;
        return true;
    }

    if(isArriveWayPoint() == true){
        getWayPoint();
    }

    if(!dwa_control(config_map))
        std::cerr << "DWA Plan is failed!!\n";

    go_v = go_v_;
    turn_v = turn_v_;

    go_v_ = 0.0;
    turn_v_ = 0.0;
    // print
    return false;
}

