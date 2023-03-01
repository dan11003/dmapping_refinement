

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"
#include "fstream"
#include "iostream"
#include "stdio.h"
#include "string.h"
#include "pcl/features/normal_3d.h"
#include "lio_sam/cloud_info.h"
#include "utility.h"



/*
std::string CreateFolder(const std::string& basePath, const std::string& prefix){
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  char buffer[32];
  std::strftime(buffer, 32, "%a_%Y.%m.%d_%H:%M:%S", ptm);
  const std::string dir = basePath + "/" + prefix +"_" + std::string(buffer) + std::string("/");
  if (boost::filesystem::create_directories(dir)){
      std::cout << "\"refinement\" - Created new output directory: "  << dir << std::endl;
  }
  return dir;
}*/

bool keep_running = true;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "refinement");
    ros::NodeHandle nh;
    lidar::Lidar lidar_param;

    std::string directory;
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    double output_downsample_size = 0.05;
    std::string loss_function = "Huber";
    bool save_Posegraph = false;
    bool save_BALM = true;
    bool save_odom = false;
    bool export_pcd = true;

    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/directory_output", directory);
    nh.getParam("/output_downsample_size", output_downsample_size);
    nh.getParam("/loss_function", loss_function);

    directory = CreateFolder(directory, "dmapping_refinement");

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);


    ros::Rate r(100); // 10 hz
    while (keep_running){
      ros::spinOnce();
      r.sleep();
    }


  return 0;
}


