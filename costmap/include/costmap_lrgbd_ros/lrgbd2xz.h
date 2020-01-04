#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class LRGBDCostMap{

public:
    LRGBDCostMap();
    ~LRGBDCostMap(void);
 
    void init(Eigen::Matrix3d camera_K, int image_height, int image_width, double resolution_size, double map_width, double map_height, bool display_costmap, int depthScale, cv::Mat T_camera2base, cv::Mat T_laser2base, double robot_radius);
    void laserToCostMap(void);
    void depthCameraToCostMap(cv::Mat depth_image);
    void obstacleMap(void);
    void inflationHighResolution(void);
    void drawCross(void);
    void drawRGBDBounds(void);
    void drawFreeArea(void);
 
    cv::Mat config_map_;

private:
    cv::Mat costmap2d_;
    Eigen::Matrix3d camera_K_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    int band_width_;
    int image_height_;
    int image_width_;
    double robot_radius_;
    double resolution_size_;
    double map_width_;
    double map_height_;
    int map_image_w_;
    int map_image_h_;
    bool display_costmap_;
    int depthScale_;
    Eigen::Matrix<double, 4, 4> T_camera2base_;
    Eigen::Matrix<double, 4, 4> T_laser2base_;
    cv::Mat T_base2world_;
    cv::Mat inflation_map_;
};