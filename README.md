<!--
 * @Author: your name
 * @Date: 2019-12-31 15:21:52
 * @LastEditTime : 2020-01-02 16:11:05
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/README.md
 -->
# orbslam_semantic_nav_ros
[![Watch the video](image/video_cover.png)](https://www.bilibili.com/video/av81398597)

# 使用说明

版本要求：

Linux:ubuntu16 
ros:kinect

### 1. 添加Vocabulary
在ros包的下新建Vocabulary文件夹，添加ORBvoc.txt词典。
```
mkdir Vocabulary
add ORBvoc.txt
```

### 2. 安装ncnn库
```
cd Thirdparty/ncnn
mkdir build&&cd build
cmake ..
make
sudo make install
sudo cp Thirdparty/ncnn/build/install/lib/libncnn.a /usr/lib
```

### 3 其他库
vtk 5
pcl 1.7
opencv 3.4
Eigen

### 4 开始编译
```
catkin_make
```
 

## 语义地图
![merge-map](image/merge_2d3d_map2.png)

