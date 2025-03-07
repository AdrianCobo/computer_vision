/*
  Code adapted from: https://github.com/jmguerreroh/computer_vision/blob/humble/include/computer_vision/CVSubscriber.hpp
  Copyright (c) 2024 José Miguel Guerrero Hernández
  Copyright (c) 2025 Adrián Cobo Merino

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

// This node recives 3 depth imgs, sync them and publish a unique pcl knowing the relative position of the cameras

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
    camera_model1_ = nullptr;
    camera_model2_ = nullptr;

    subscription_info1_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info1", 1,
      std::bind(&CVSubscriber::topic_callback_info1, this, _1));
    
    subscription_info2_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info2", 1,
      std::bind(&CVSubscriber::topic_callback_info2, this, _1));

    subscription_depth1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in1", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in2", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(1000000), *subscription_depth1_, *subscription_depth2_);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2));
  }

private:
  bool check_subscription_count_;

  // TODO: revisar estos 3 callbacks para no repetir código.
  void topic_callback_info1(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info 1 received");

    camera_model1_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model1_->fromCameraInfo(*msg);

    //subscription_info1_.reset();
  }

    void topic_callback_info2(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info 2 received");

    camera_model2_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model2_->fromCameraInfo(*msg);

    //subscription_info2_.reset();
  }

    void depth2pcl(const cv::Mat& input,  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model, pcl::PointCloud<pcl::PointXYZ>& final_pcl)
  {
    cv::Mat intrinsic_marix = (cv::Mat)camera_model->intrinsicMatrix();
    float fx, fy, cx, cy;
    
    fx = (float)intrinsic_marix.at<double>(0, 0);
    fy = (float)intrinsic_marix.at<double>(1, 1);
    cx = (float)intrinsic_marix.at<double>(0, 2);
    cy = (float)intrinsic_marix.at<double>(1, 2);

    // Recorrer la imagen fila por fila
    #pragma omp parallel for
    for (int row = 0; row < input.rows; row += 4) {
      const float* ptr = input.ptr<float>(row);
      std::vector<pcl::PointXYZ> local_points;  // Cada hilo usa un vector local

      for (int col = 0; col < input.cols; col += 4) {
        float d = ptr[col] / 1000.0f;
        if (!std::isfinite(d)) continue;

        float x_3d = (col - cx) * d / fx;
        float y_3d = (row - cy) * d / fy;
        float z_3d = d;

        local_points.emplace_back(x_3d, y_3d, z_3d);
      }

      #pragma omp critical
      final_pcl.insert(final_pcl.end(), local_points.begin(), local_points.end());
    }
  }

  void topic_callback_multi(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg1,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg2)
  {
    // Check if camera model has been received
    if (camera_model1_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model 1 not yet available");
      return;
    }

    if (camera_model2_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model 2 not yet available");
      return;
    }

    // Check if depth image has been received
    if ((image_depth_msg2->encoding != "16UC1" && image_depth_msg2->encoding != "32FC1") ||
        (image_depth_msg1->encoding != "16UC1" && image_depth_msg1->encoding != "32FC1")) {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
    cv_bridge::CvImagePtr image_depth_ptr1, image_depth_ptr2;
    try {
      image_depth_ptr1 = cv_bridge::toCvCopy(
          *image_depth_msg1, 
          sensor_msgs::image_encodings::TYPE_32FC1);
      image_depth_ptr2 = cv_bridge::toCvCopy(
        *image_depth_msg2,
        sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> c1, c2;
    
    depth2pcl(image_depth_ptr1->image, camera_model1_, c1);
    depth2pcl(image_depth_ptr1->image, camera_model2_, c2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c1_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(c1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2_ptr =  std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(c2);;

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

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_depth1_, subscription_depth2_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info1_, subscription_info2_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model1_, camera_model2_;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_