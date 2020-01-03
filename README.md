<!--
 * @Author: your name
 * @Date: 2019-12-31 15:21:52
 * @LastEditTime : 2020-01-03 15:10:34
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
pangolin
octomap

### 4 安装语音功能
- step1 在科大讯飞注册账户，修改appid
- step2 安装相关库
```
sudo apt-get install libasound2-dev #asound库
sudo apt-get install mplayer #mplayer工具
```
- step3 在科大讯飞官网下载SDK，在fileroot/libs/x64中将libmsc.so文件复制到/usr/local/lib中

### 5 安装aip相关库
```
sudo apt-get install libjsoncpp-dev
sudo apt-get install openssl
sudo apt-get install curl
```

### 6 开始编译
```
catkin_make
```
 
 
## 语义地图
![merge-map](image/merge_2d3d_map2.png)

