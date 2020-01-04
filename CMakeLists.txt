cmake_minimum_required(VERSION 2.8.3)
project(slam_semantic_nav_ros)

SET(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

#编译模式 使用 IF(NOT ) ENDIF 放置重复设置
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
# 显示 编译模式信息
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")

# 检查c++11或者 c++0x 编译支持  Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 附加 模块cmakeList.txt
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_filters
  image_transport
  tf
  geometry_msgs
  tf2 
  image_geometry
  visualization_msgs 
)
 

#opencv  模块 版本大于 2.4.3
#set(OpenCV_DIR "~/code/catkin_ws/other/opencv-3.4.3/build")
find_package(OpenCV QUIET)
#message(FATAL_ERROR ${OpenCV_LIBS})
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(PkgConfig)

 # 矩阵 Eigen3
find_package(Eigen3 REQUIRED)
# 可视化gui Pangolin
find_package(Pangolin REQUIRED)

# 点云显示 adding for point cloud viewer and mapper
find_package( PCL REQUIRED )

# octomap==================
set(OCTOMAP_INCLUDE_DIRS "/opt/ros/kinetic/include")
set(OCTOMAP_LIBRARIES "/opt/ros/kinetic/lib/liboctomap.so /opt/ros/kinetic/lib/liboctomath.so")
#FIND_PACKAGE( octomap REQUIRED )



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Arrived.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
std_msgs 
)


###########
## Build ##
###########


#包含库文件
include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/speech
${PROJECT_SOURCE_DIR}/costmap/include
${PROJECT_SOURCE_DIR}/aip-cpp/include
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/orbslam/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}# 可视化gui
${PCL_INCLUDE_DIRS}#点云库
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/include          # ncnn 目标检测库
${OCTOMAP_INCLUDE_DIRS}  # octomap 
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


# 自建库生成路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 创建共享库 SHARED 动态链接库
add_library(${PROJECT_NAME} SHARED
orbslam/src/System.cc
orbslam/src/Tracking.cc
orbslam/src/LocalMapping.cc
orbslam/src/LoopClosing.cc
orbslam/src/ORBextractor.cc
orbslam/src/ORBmatcher.cc
orbslam/src/FrameDrawer.cc
orbslam/src/Converter.cc
orbslam/src/MapPoint.cc
orbslam/src/KeyFrame.cc
orbslam/src/Map.cc       # add 保存、载入orbslam 地图点、关键帧、最小生成树  
orbslam/src/MapDrawer.cc # add 创建、显示、保存、载入octomap
orbslam/src/Optimizer.cc
orbslam/src/PnPsolver.cc
orbslam/src/Frame.cc
orbslam/src/KeyFrameDatabase.cc
orbslam/src/Sim3Solver.cc
orbslam/src/Initializer.cc
orbslam/src/pointcloudmapping.cc # pcl点云建图线程，已丢弃==
orbslam/src/Geometry.cc          # 多视角几何判断动静mask     较慢
orbslam/src/Flow.cc              # 光流判断动静mask           快
orbslam/src/ObjectDatabase.cc    # 语义目标数据库
orbslam/src/Merge2d3d.cc         # 2d框和点云获取3d语义对象 深度信息滤波方法       快
orbslam/src/MergeSG.cc           # 2d框和点云获取3d语义对象 点云分割聚类求交集方法  稍慢
orbslam/src/Detector.cc          # 2d目标检测 / 实例分割 / 语义分割
orbslam/src/RunDetect.cc         # 2d目标检测 运行线程 类 ，为了并行==
orbslam/src/Viewer.cc
)  


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES orbslam_semantic_nav_ros
 CATKIN_DEPENDS roscpp rospy message_filters
#  costmap_lrgbd_ros
 message_generation
 image_transport 
 tf
 geometry_msgs
 DEPENDS system_lib
)
 
set(NCNN_TARGET_LIB ncnn) # ncnn 库
 
# 连接库文件
target_link_libraries(
${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so 
${PCL_LIBRARIES}
/opt/ros/kinetic/lib/liboctomap.so
        /opt/ros/kinetic/lib/liboctomath.so
#${OCTOMAP_LIBRARIES} # octomap

${NCNN_TARGET_LIB}
-fopenmp
)

add_executable(kinect_camera_node src/kinect_camera_node.cpp src/Node.cc src/RGBDNode.cpp)

target_link_libraries(kinect_camera_node  
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/lib/libslam_semantic_nav_ros.so
${PCL_LIBRARIES}  
)

add_executable(tts_subscribe_node speech/src/tts_subscribe_node.cpp)
target_link_libraries(tts_subscribe_node ${catkin_LIBRARIES} libmsc.so -ldl -lpthread)

add_executable(iat_publish_node speech/src/iat_publish_node.cpp speech/src/speech_recognizer.c speech/src/linuxrec.c)
target_link_libraries(iat_publish_node ${catkin_LIBRARIES} libmsc.so -ldl -lpthread -lm -lrt -lasound)

add_executable(gesture_pub_node aip-cpp/src/gesture_pub_node.cpp )
target_link_libraries(gesture_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS} -lcurl -ljsoncpp -lcrypto)

add_executable(face_pub_node aip-cpp/src/face_pub_node.cpp )
target_link_libraries(face_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS} -lcurl -ljsoncpp -lcrypto)

add_executable(rgbd_costmap_node costmap/src/lrgbd_costmap_node.cpp costmap/src/lrgbd2xz.cpp costmap/src/dwa_planning.cpp) 
target_link_libraries(rgbd_costmap_node  ${catkin_LIBRARIES} ${OpenCV_LIBS})
 
add_executable(record_pose_node costmap/src/record_path_node.cpp) 
target_link_libraries(record_pose_node  ${catkin_LIBRARIES} ${OpenCV_LIBS})