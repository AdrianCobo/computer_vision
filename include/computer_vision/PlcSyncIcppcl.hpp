/*
  Code adapted from: https://github.com/jmguerreroh/computer_vision/blob/humble/include/computer_vision/CVSubscriber.hpp
  Copyright (c) 2024 José Miguel Guerrero Hernández
  Copyright (c) 2025 Adrián Cobo Merino

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

// This node recives 2 pcls from input topics and execute ICP

#ifndef INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_
#define INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_

#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <omp.h>
#include <Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

namespace computer_vision
{

using std::placeholders::_1;
using std::placeholders::_2;

class CVSubscriber : public rclcpp::Node
{
public:
  CVSubscriber()
  : Node("depth_sync")
  {
    this->declare_parameter("check_subscription_count", false);
    this->get_parameter("check_subscription_count", check_subscription_count_);

    subscription_pointcloud_1 =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, "/pointcloud_in1", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_pointcloud_2 =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    this, "/pointcloud_in2", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(1000000), *subscription_pointcloud_1, *subscription_pointcloud_2);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2));
  }

private:
  bool check_subscription_count_;

  void topic_callback_multi(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & image_depth_msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & image_depth_msg2)
  {
    pcl::PointCloud<pcl::PointXYZ> c1, c2;
    
    pcl::fromROSMsg(*image_depth_msg1, c1);
    pcl::fromROSMsg(*image_depth_msg2, c2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c1_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(c1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2_ptr =  std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(c2);
    std::vector<int> indices; // Indices of valid points
    pcl::removeNaNFromPointCloud(*c1_ptr, *c1_ptr, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource (c1_ptr);
    icp.setInputTarget (c2_ptr);
    
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (200);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.001);
    
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    icp.align (cloud_source_registered);

    std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> subscription_pointcloud_1, subscription_pointcloud_2;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_