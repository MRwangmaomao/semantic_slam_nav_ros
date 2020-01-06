/*
 * @Author: 王培荣
 * @Date: 2019-12-29 11:15:26
 * @LastEditTime : 2020-01-06 16:51:58
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/src/RGBDNode.cpp
 */
  
#include "Node.h"
#include <ros/ros.h>
#include <ros/console.h> 
#include <iostream>
 
Node::Node (ros::NodeHandle &node_handle, std::string config_file_path) {
  ROS_INFO_STREAM("Setting file path is: " << config_file_path);
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle));
  close_system_ = false; 
  cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
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
  fsSettings["name_of_node"] >> name_of_node_;
  fsSettings["rospackage_path"] >> rospackage_path_;
  fsSettings["color_img_topic"] >> color_img_topic_;
  fsSettings["depth_img_topic"] >> depth_img_topic_;
  fsSettings["map_frame_id_param"] >> map_frame_id_param_;
  fsSettings["camera_frame_id_param"] >> camera_frame_id_param_;
  fsSettings["map_file_name_param"] >> map_file_name_param_;
  map_file_name_param_ = rospackage_path_ + map_file_name_param_;
  fsSettings["voc_file_name_param"] >> voc_file_name_param_;
  voc_file_name_param_ = rospackage_path_ + voc_file_name_param_;
  fsSettings["settings_file_name_param"] >> settings_file_name_param_;
  fsSettings["folder_path"] >> folder_path_;
  settings_file_name_param_ = rospackage_path_ + settings_file_name_param_;
  int if_load_map_param = fsSettings["load_map_param"];
  if(if_load_map_param){
    load_map_param_ = true;
  } 
  else{
    load_map_param_ = false;
  }
  int if_publish_pointcloud_param = fsSettings["publish_pointcloud_param"];
  if(if_publish_pointcloud_param){
    publish_pointcloud_param_ = true;
  } 
  else{
    publish_pointcloud_param_ = false;
  }
  int if_publish_pose_param = fsSettings["publish_pose_param"];
  if(if_publish_pose_param){
    publish_pose_param_ = true;
  } 
  else{
    publish_pose_param_ = false;
  }
  min_observations_per_point_ = fsSettings["min_observations_per_point"];
 
  node_handle_ = node_handle;
  // min_observations_per_point_ = 2;
 

  // service_server_ = node_handle_.advertiseService(name_of_node_+"/save_map", &Node::SaveMapSrv, this); // 订阅服务，是否保存地图指令

  // //Setup dynamic reconfigure
  // dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback; // 动态参数配置
  // dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  // dynamic_param_server_.setCallback(dynamic_param_callback);

  rendered_image_publisher_ = it_->advertise (name_of_node_+"/debug_image", 1);// 注册发布调试图像
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1); // 注册发布地图点云
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1); // 注册发布位姿
  }

  is_arrived_sub_ = node_handle_.subscribe("/isArrived", 10, &Node::isArrivedCallback, this);

}

void Node::init_slam()
{ 

  orb_slam_ = new ORB_SLAM2::System (voc_file_name_param_, settings_file_name_param_, ORB_SLAM2::System::RGBD, folder_path_, true, load_map_param_); // 初始化slam系统
}

Node::~Node () { 

}

void Node::isArrivedCallback(const orbslam_semantic_nav_ros::Arrived::ConstPtr arrived_msg){
  if(arrived_msg->is_arrived == 1){
    orb_slam_->Shutdown(); // 关闭系统
    // // Save camera trajectory
    orb_slam_->SaveTrajectoryTUM("CameraTrajectory.txt");
    orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); // 保存关键帧轨迹
    orb_slam_->SaveMap("map.bin");
    orb_slam_->SaveOctomap("octomap.ot"); // 保存octomap地图
    close_system_ = true;
    std::cout << "关闭SLAM系统!" << std::endl;
    delete orb_slam_; // 删除指针对象
    ros::shutdown();
  } 
}

bool Node::Update() {
  // cv::Mat position = orb_slam_->GetCurrentPosition(); // 通过orb_slam 对象获取当前姿态 
  cv::Mat position;
  if (!position.empty()) {
    PublishPositionAsTransform (position); // 发布TF

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);  // 发布位姿
    }
  }

  // PublishRenderedImage (orb_slam_->DrawCurrentFrame()); // 发布当前帧的图像（带特征点）

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetTrackedMapPoints()); // 发布地图点云
  }
  return close_system_;
}


void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  // sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points); // 将map point转为pointcloud2格式
  // map_points_publisher_.publish (cloud); // 发布点云
}

/**
 * @brief 发布camera在map中的tf
 * 
 * @param position 
 */
void Node::PublishPositionAsTransform (cv::Mat position) {
  tf::Transform transform = TransformFromMat (position);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
}

// 将Mat转为pseStamped并发布 
void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf::Transform grasp_tf = TransformFromMat (position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg); 
  pose_publisher_.publish(pose_msg);
}

/**
 * @brief 发布调试图像
 * 
 * @param image 
 */
void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}

/**
 * @brief 将ORBSLAM中的cv::Mat转为tf::Transform
 * 
 * @param position_mat 
 * @return tf::Transform 
 */
tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

/**
 * @brief ORB_SLAM地图点云转为PointCloud2点云
 * 
 * @param map_points 
 * @return sensor_msgs::PointCloud2 
 */
sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1; // 默认高度为1
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for(unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));// 起始地址  内存地址  大小
    }
  }

  return cloud;
}


// void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
//   // orb_slam_->EnableLocalizationOnly (config.localize_only);
//   // min_observations_per_point_ = config.min_observations_for_ros_map;

//   // if (config.reset_map) {
//   //   orb_slam_->Reset();
//   //   config.reset_map = false;
//   // }

//   // orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
// }


// bool Node::SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) {
//   res.success = orb_slam_->SaveMap(req.name); // 保存地图算子

//   if (res.success) {
//     ROS_INFO_STREAM ("Map was saved as " << req.name);
//   } else {
//     ROS_ERROR ("Map could not be saved.");
//   }

//   return res.success;
// }