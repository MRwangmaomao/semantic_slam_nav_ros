<!--
 * @Author: 王培荣
 * @Date: 2019-12-31 15:21:52
 * @LastEditTime : 2020-01-09 23:57:19
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/orbslam_semantic_nav_ros/README.md
 -->
# orbslam_semantic_nav_ros

TODO：
   
1. 常用语音对话与语音唤醒功能

2. 添加基于二维码的全局定位误差矫正功能

3. 待定

---
演示1：

[![Watch the video](image/cover1.png)](https://www.bilibili.com/video/av81958116)

---

演示2：

[![Watch the video](image/cover2.png)](https://www.bilibili.com/video/av81398597)

---

# 安装说明

系统版本要求：

- Linux:ubuntu16

- ros:kinect

已测试硬件：

- KinectV2相机

- realsense D435,D400相机

### (1) 添加Vocabulary
在ros包的下新建Vocabulary文件夹，添加ORBvoc.txt词典。
```
mkdir Vocabulary
add ORBvoc.txt
```

### (2) 安装腾讯ncnn库
```
cd Thirdparty/ncnn
mkdir build&&cd build
cmake ..
make
sudo make install
sudo cp Thirdparty/ncnn/build/install/lib/libncnn.a /usr/lib
```

### (3) 其他库

vtk 5

pcl 1.7

opencv 3.4

Eigen

[pangolin](https://github.com/stevenlovegrove/Pangolin)

octomap

### (4) 安装科大讯飞语音相关库
- step1 在科大讯飞注册账户，修改speech/src中appid参数
- step2 安装相关库
```
sudo apt-get install libasound2-dev #asound库
sudo apt-get install mplayer #mplayer工具
```
- step3 在科大讯飞官网下载SDK，在fileroot/libs/x64中将libmsc.so文件复制到/usr/local/lib中


### (5) 安装百度aip相关库
```
sudo apt-get install ros-kinetic-image-view
sudo apt-get install libjsoncpp-dev
sudo apt-get install openssl
sudo apt-get install curl
```

修改aip-cpp/src中的appid,AK,SK

### (6) 开始下载源码进行编译
```
mkdir -p catkin_ws/src
git clone git@github.com:MRwangmaomao/semantic_slam_nav_ros.git
cd ..
catkin_make
```

### (7) 修改config文件夹下的setting.yaml文件和对应的相机的配置文件、DWA配置文件

[配置文件说明](config/README.md)

修改所有的rospackage_path参数，设置为自己对应存储的路径。

修改color_img_topic和depth_img_topic

### (8) 相关公开数据集下载

[OpenLORIS-Scene Dataset](https://lifelong-robotic-vision.github.io/dataset/scene)

# 动态环境的ORBSLAM
![merge-map](image/dynamic_pic2.png) 


![point_feature](image/pointfeature1.png)

# 语义地图

```
roslaunch slam_semantic_nav_ros ORB_Semantic_Nav.launch
```
![merge-map](image/ssd_map1.png)

![ssd_detect](image/ssd_detect1.png)


# 三维重构
![三维重构](image/3d_restructure1.png)

# 代价地图与导航
![costmap](image/costmap1.png)

# 语音对话
```
roslaunch slam_semantic_nav_ros gesture_speak.launch 
```

# 手势识别

![手势1](image/gesture1.png) ![手势2](image/gesture2.png)

```
roslaunch slam_semantic_nav_ros gesture_speak.launch
```

# 参考

[ORBSLAM](https://github.com/raulmur/ORB_SLAM2)

[ORB_SLAM2_SSD_Semantic](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic)

[百度AI开放平台](https://ai.baidu.com/)

[科大讯飞语音](https://www.xfyun.cn/)
