
#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <vector>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h> 
#include <tf/transform_broadcaster.h> 
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "costmap_lrgbd_ros/lrgbd2xz.h"
#include "costmap_lrgbd_ros/dwa_planning.h"
#include "costmap_lrgbd_ros/Arrived.h"

#define RAD2DEG(x) ((x)*180./M_PI) 

LRGBDCostMap lrgbd_tmap;
DWAPlanning dwa_planer;
Eigen::Matrix4d robot_pose;
ros::Publisher speed_pub;
ros::Publisher all_marker_pub;
ros::Publisher dest_marker_pub;
ros::Publisher isArrived_pub;
long int robot_pose_id;
std::string all_waypoint_file;
bool pub_waypoint_marker_flag;

template<typename T>
T getOption(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}



void img_callback(const sensor_msgs::ImageConstPtr &depth)
{  
    try
    {
        double go_v = 0.0, turn_v = 0.0;
        geometry_msgs::Twist pub_speed;
        costmap_lrgbd_ros::Arrived pub_arrived;
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(depth);
        lrgbd_tmap.depthCameraToCostMap(cv_image->image);
        if(dwa_planer.move(robot_pose_id, robot_pose, lrgbd_tmap.config_map_, go_v, turn_v)){
            pub_speed.linear.x = 0;
            pub_speed.angular.z = 0;
            // ROS_INFO_STREAM("go_v: " << go_v <<"    turn_v: " << turn_v);
            speed_pub.publish(pub_speed);
            pub_arrived.is_arrived = 1;
            isArrived_pub.publish(pub_arrived);
            ros::shutdown();
            return;
        }
        pub_speed.linear.x = go_v;
        pub_speed.angular.z = turn_v;
        // ROS_INFO_STREAM("go_v: " << go_v <<"    turn_v: " << turn_v);
        speed_pub.publish(pub_speed);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } 
} 

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}

void waypoints_pub(std::string waypoints_file_path, int robot_start_x, int robot_start_y){
    
    visualization_msgs::MarkerArray waypoints_markerArray;
    std::ifstream waypoint_file(waypoints_file_path);
    ROS_INFO_STREAM("Display waypoints on rviz.");
    std::string temp;
    double waypoint_x, waypoint_y;
    std::string temp_w;
    int k = 0;
    while(getline(waypoint_file,temp)) //按行读取字符串 
	{  
        std::stringstream input(temp);
        input>>temp_w;
        input>>temp_w; 
        waypoint_x = atof(temp_w.c_str());
        input>>temp_w; 
        waypoint_y = atof(temp_w.c_str()); 
        visualization_msgs::Marker waypoints_marker;
        waypoints_marker.header.frame_id = "/map";
        waypoints_marker.header.stamp = ros::Time::now();
        waypoints_marker.action = visualization_msgs::Marker::ADD;
        waypoints_marker.type = visualization_msgs::Marker::CUBE;
        waypoints_marker.id = k;
        waypoints_marker.ns = "basic_shapes";
        waypoints_marker.scale.x = 0.1;
        waypoints_marker.scale.y = 0.1;
        waypoints_marker.scale.z = 0.1;
        waypoints_marker.color.b = 1.0;
        waypoints_marker.color.g = 0.0;
        waypoints_marker.color.r = 0.0;
        waypoints_marker.color.a = 1.0;
        waypoints_marker.lifetime = ros::Duration(); 
        geometry_msgs::Pose pose;
        pose.position.x = waypoint_x;
        pose.position.y = waypoint_y;
        pose.position.z = 0;
        pose.orientation.w = 1.0;
        waypoints_marker.pose = pose;
        std::ostringstream str;
        str<<k;
        waypoints_marker.text=str.str();
        waypoints_markerArray.markers.push_back(waypoints_marker); 
        while(all_marker_pub.getNumSubscribers() < 1){
            if (!ros::ok()){
    //             return 0; 
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        all_marker_pub.publish(waypoints_markerArray); 
        k++;
	} 
	waypoint_file.close();   
}


void robot_pose_callback(const tf2_msgs::TFMessage::ConstPtr msg){
    
    robot_pose_id++;
    robot_pose(0,3) = msg->transforms.at(0).transform.translation.x;
    robot_pose(1,3) = msg->transforms.at(0).transform.translation.y;
    robot_pose(2,3) = msg->transforms.at(0).transform.translation.z; 
    Eigen::Quaterniond robot_Q(msg->transforms.at(0).transform.rotation.w, msg->transforms.at(0).transform.rotation.x, 
        msg->transforms.at(0).transform.rotation.y, msg->transforms.at(0).transform.rotation.z);
    robot_pose.block(0,0,3,3) = robot_Q.toRotationMatrix();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(robot_pose(0,3), robot_pose(1,3), robot_pose(2,3)));
    tf::Quaternion q(msg->transforms.at(0).transform.rotation.x, msg->transforms.at(0).transform.rotation.y,
                    msg->transforms.at(0).transform.rotation.z,msg->transforms.at(0).transform.rotation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->transforms.at(0).header.stamp.now(), "/map", "/base_link"));
    if(!pub_waypoint_marker_flag){
        pub_waypoint_marker_flag = true;
        waypoints_pub(all_waypoint_file, robot_pose(0,3), robot_pose(1,3));
    }
    dest_marker_pub.publish(dwa_planer.dest_waypoint_pub());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lrgbd_costmap");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::cout << "\n-------------------------- start --------------------------\n" << std::endl;
    
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file;
    std::string dwa_file; 
    robot_pose << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
    robot_pose_id = 0;
    pub_waypoint_marker_flag = false;
    if(argc >=2){
        config_file = argv[1]; 
    }
    else{
        std::cout<<"args too small.\n";
        exit(0);
    } 
    
    ROS_INFO_STREAM("Setting file path is: " << config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    fsSettings["waypoints_file"] >> all_waypoint_file;
    fsSettings["dwa_file"] >> dwa_file;
    double fx = fsSettings["fx"];
    double fy = fsSettings["fy"];
    double cx = fsSettings["cx"];
    double cy = fsSettings["cy"]; 
    
    Eigen::Matrix3d camera_K; 
    camera_K << fx, 0, cx, fy, cy, 0, 0, 0, 1;
    int image_height = fsSettings["image_height"];
    int image_width = fsSettings["image_width"];
    
    int depthScale = fsSettings["depthScale"];
    double resolution_size = fsSettings["resolution_size"];
    double map_width = fsSettings["map_width"];
    double map_height = fsSettings["map_height"];
    double robot_radius = fsSettings["robot_radius"];
    
    // bool display_costmap = fsSettings["display_costmap"];
    bool display_costmap = false;
    
    cv::Mat T_lidar2base, Tdepth2base;
    fsSettings["DEPTH_BASE"] >> Tdepth2base;
    fsSettings["LIDAR_BASE"] >> Tdepth2base; 
 
    lrgbd_tmap.init(camera_K, image_height, image_width, resolution_size, map_width, map_height, display_costmap, depthScale, Tdepth2base, Tdepth2base, robot_radius);
    dwa_planer.init(dwa_file, depthScale, camera_K, map_width, map_height, resolution_size);

    std::string depth_topic = fsSettings["depth_topic"];
    std::string lidar_topic = fsSettings["lidar_topic"];
    std::string robot_pose_topic = fsSettings["robot_pose_topic"];
    std::string camera_info = fsSettings["camera_info"];
    
    ros::Subscriber sub_img = nh.subscribe(depth_topic, 100, img_callback);
    ros::Subscriber sub_laser = nh.subscribe(lidar_topic, 100, lidar_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe(robot_pose_topic, 100, robot_pose_callback);
    speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    isArrived_pub = nh.advertise<costmap_lrgbd_ros::Arrived>("/isArrived", 10);
    all_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/WayPoints",10);
    dest_marker_pub = nh.advertise<visualization_msgs::Marker>("/DestWayPoint",10);

    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
