/*
  Code adapted from: https://github.com/jmguerreroh/computer_vision/blob/humble/include/computer_vision/CVSubscriber.hpp
  Copyright (c) 2024 José Miguel Guerrero Hernández
  Copyright (c) 2025 Adrián Cobo Merino

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

// This node reads 2 .pcd and execute

#ifndef INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_
#define INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_



#include "rclcpp/rclcpp.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/Dense>

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace computer_vision
{

using namespace std::chrono_literals;

class CVSubscriber : public rclcpp::Node
{
public:
  CVSubscriber()
  : Node("depth_sync")
  {
    this->declare_parameter("pcl1", "/home/adrianco/Desktop/ros2_ws/src/map_lab1.pcd");
    this->get_parameter("pcl1", pcl1_path_);
    this->declare_parameter("pcl2", "/home/adrianco/Desktop/ros2_ws/src/map_lab2.pcd");
    this->get_parameter("pcl2", pcl2_path_);

    publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map1",
      rclcpp::SensorDataQoS().reliable());

    publisher_pointcloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map2",
      rclcpp::SensorDataQoS().reliable());

    timer_ = this->create_wall_timer(
      500ms, std::bind(&CVSubscriber::icp_callback, this));
  }

private:
  std::string pcl1_path_, pcl2_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_, publisher_pointcloud2_;

  void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);  // Número de vecinos a analizar
    sor.setStddevMulThresh(1.0);  // Umbral de desviación estándar
    sor.filter(*cloud);
  }

  void removeOutliersByRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.05);  // Radio de búsqueda en metros
    outrem.setMinNeighborsInRadius(5);  // Mínimo de vecinos requeridos
    outrem.filter(*cloud);
  }

  void icp_callback()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr c1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout <<"Hola"<< std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/adrianco/Desktop/ros2_ws/src/p1_high_precision_1.pcd", *c1_ptr) == -1) {
      PCL_ERROR("No se pudo leer el archivo PCD 1\n");
      return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/adrianco/Desktop/ros2_ws/src/p1_high_precision_2.pcd", *c2_ptr) == -1) {
      PCL_ERROR("No se pudo leer el archivo PCD 2\n");
      return;
    }
    
    // PCL 1
    removeOutliers(c1_ptr);
    removeOutliersByRadius(c1_ptr);

    sensor_msgs::msg::PointCloud2 out_pointcloud;
    pcl::toROSMsg(*c1_ptr, out_pointcloud);
    out_pointcloud.header.frame_id = "map";
    out_pointcloud.header.stamp = this->get_clock()->now();

    // Publish the data
    publisher_pointcloud_->publish(out_pointcloud);

    // PCL 2
    removeOutliers(c2_ptr);
    removeOutliersByRadius(c2_ptr);

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource (c1_ptr);
    icp.setInputTarget (c2_ptr);
    
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (200000);
    // Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon (0.001);
    
    // Perform the alignment
    // pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    // icp.align (cloud_source_registered);

    std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    pcl::toROSMsg(*c2_ptr, out_pointcloud);
    out_pointcloud.header.frame_id = "map";
    out_pointcloud.header.stamp = this->get_clock()->now();

    // Publish the data
    publisher_pointcloud2_->publish(out_pointcloud);
  }
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_