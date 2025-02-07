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

//un solo bucle:

namespace computer_vision
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

int N_CAMS = 3;

class CVGroup
{
public:
  CVGroup(cv::Mat image_depth1, cv::Mat image_depth2, cv::Mat image_depth3)
  {
    image_depth1_ = image_depth1;
    image_depth2_ = image_depth2;
    image_depth3_ = image_depth3;
  }
  cv::Mat getImageDepth1() {return image_depth1_;}
  cv::Mat getImageDepth2() {return image_depth2_;}
  cv::Mat getImageDepth3() {return image_depth3_;}

private:
  cv::Mat image_depth1_;
  cv::Mat image_depth2_;
  cv::Mat image_depth3_;
};

class DepthSync
{
public:
  DepthSync(cv::Mat image_depth1, cv::Mat image_depth2, cv::Mat image_depth3)
  {
    image_depth_1 = image_depth1;
    image_depth_2 = image_depth2;
    image_depth_3 = image_depth3;
  }
  cv::Mat getImageDepth1() {return image_depth_1;}
  cv::Mat getImageDepth2() {return image_depth_2;}
  cv::Mat getImageDepth3() {return image_depth_3;}

private:
  cv::Mat image_depth_1;
  cv::Mat image_depth_2;
  cv::Mat image_depth_3;
};

class CVSubscriber : public rclcpp::Node
{
public:
  CVSubscriber()
  : Node("depth_sync")
  {
    this->declare_parameter("check_subscription_count", false);
    this->get_parameter("check_subscription_count", check_subscription_count_);

    subscription_info1_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info1", 1,
      std::bind(&CVSubscriber::topic_callback_info1, this, _1));
    
    subscription_info2_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info2", 1,
      std::bind(&CVSubscriber::topic_callback_info2, this, _1));

    subscription_info3_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info3", 1,
      std::bind(&CVSubscriber::topic_callback_info3, this, _1));

    subscription_depth1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in1", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in2", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth3_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in3", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy1>>(
      MySyncPolicy1(100000), *subscription_depth1_, *subscription_depth2_, *subscription_depth3_);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2, _3));

    publisher_pcl = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_sync",
      rclcpp::SensorDataQoS().reliable());
  }

private:
  bool check_subscription_count_;

  // TODO: revisar estos 3 callbacks para no repetir código.
  void topic_callback_info1(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info 1 received");

    camera_model1_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model1_->fromCameraInfo(*msg);

    subscription_info1_ = nullptr;
  }

    void topic_callback_info2(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info 2 received");

    camera_model2_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model2_->fromCameraInfo(*msg);

    subscription_info2_ = nullptr;
  }

    void topic_callback_info3(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info 3 received");

    camera_model3_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model3_->fromCameraInfo(*msg);

    subscription_info3_ = nullptr;
  }

  void y_rotation(pcl::PointCloud<pcl::PointXYZ>& input_pcl, pcl::PointCloud<pcl::PointXYZ>& final_pcl, Eigen::Vector3f& translation, const double& angle)
  {
    // Crear la matriz de transformación (rotación en el eje y)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = translation;
    transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));

    pcl::transformPointCloud(input_pcl, input_pcl, transform);
    final_pcl.insert(final_pcl.end(), input_pcl.begin(), input_pcl.end());
  }

  void depth2pcl(pcl::PointCloud<pcl::PointXYZ>& final_pcl)
  {
    cv::Mat intrinsic_marix_1 = (cv::Mat)camera_model1_->intrinsicMatrix();
    cv::Mat intrinsic_marix_2 = (cv::Mat)camera_model2_->intrinsicMatrix();
    cv::Mat intrinsic_marix_3 = (cv::Mat)camera_model3_->intrinsicMatrix();

    float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, fx3, fy3, cx3, cy3;
    
    fx1 = (float)intrinsic_marix_1.at<double>(0, 0);
    fy1 = (float)intrinsic_marix_1.at<double>(1, 1);
    cx1 = (float)intrinsic_marix_1.at<double>(0, 2);
    cy1 = (float)intrinsic_marix_1.at<double>(1, 2);

    fx2 = (float)intrinsic_marix_2.at<double>(0, 0);
    fy2 = (float)intrinsic_marix_2.at<double>(1, 1);
    cx2 = (float)intrinsic_marix_2.at<double>(0, 2);
    cy2 = (float)intrinsic_marix_2.at<double>(1, 2);

    fx3 = (float)intrinsic_marix_3.at<double>(0, 0);
    fy3 = (float)intrinsic_marix_3.at<double>(1, 1);
    cx3 = (float)intrinsic_marix_3.at<double>(0, 2);
    cy3 = (float)intrinsic_marix_3.at<double>(1, 2);

    Eigen::Vector3f translation_cam2(-1.0, 0.0, -1.0);  // Traslación en X, Y, Z
    Eigen::Vector3f translation_cam3(-0.0, 0.0, -2.0);  // Traslación en X, Y, Z

    float theta = - M_PI / 2;  // 45 grados en radianes
    Eigen::Matrix3f rotationY;
    rotationY << cos(theta),  0, sin(theta),
                 0,           1, 0,
                 -sin(theta),  0, cos(theta);

    float theta2 = - M_PI / 2;  // 45 grados en radianes
    Eigen::Matrix3f rotationY2;
    rotationY2 << cos(theta2),  0, sin(theta2),
                 0,           1, 0,
                 -sin(theta2),  0, cos(theta2);

    // Recorrer la imagen fila por fila
    #pragma omp parallel for
    for (int row = 0; row < image_depth_ptr1->image.rows; ++row) {
      const float* ptr = image_depth_ptr1->image.ptr<float>(row);
      const float* ptr2 = image_depth_ptr2->image.ptr<float>(row);
      const float* ptr3 = image_depth_ptr3->image.ptr<float>(row);
      std::vector<pcl::PointXYZ> local_points;  // Cada hilo usa un vector local

      for (int col = 0; col < image_depth_ptr1->image.cols; ++col) {
        float d = ptr[col] / 1000.0f;
        if (std::isfinite(d)){
          float x_3d = (col - cx1) * d / fx1;
          float y_3d = (row - cy1) * d / fy1;
          float z_3d = d;
          
          final_pcl.push_back(pcl::PointXYZ(x_3d, y_3d, z_3d));
        }

        // no estas cambiando de imagen de profundidad
        d = ptr2[col] / 1000.0f;
        if (std::isfinite(d)){
          float x_3d = (col - cx2) * d / fx2 + translation_cam2.x();
          float y_3d = (row - cy2) * d / fy2;
          float z_3d = d + translation_cam2.z();

          Eigen::Vector3f point_vec(x_3d, y_3d, z_3d);
          Eigen::Vector3f transformed_point = rotationY * point_vec;
          final_pcl.push_back(pcl::PointXYZ(transformed_point.x(), transformed_point.y(), transformed_point.z()));
        }

        // no estas cambiando de imagen de profundidad
        d = ptr3[col] / 1000.0f;
        if (!std::isfinite(d)){
          float x_3d = (col - cx3) * d / fx3;
          float y_3d = (row - cy3) * d / fy3;
          float z_3d = d + translation_cam3.z();
          
          Eigen::Vector3f point_vec(x_3d, y_3d, z_3d);
          Eigen::Vector3f transformed_point = rotationY2 * point_vec;
          final_pcl.push_back(pcl::PointXYZ(transformed_point.x(), transformed_point.y(), transformed_point.z()));
        }
      }
    }
  }

  void topic_callback_multi(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg1,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg2,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg3)
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

    if (camera_model3_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model 3 not yet available");
      return;
    }

    // Check if depth image has been received
    if ((image_depth_msg2->encoding != "16UC1" && image_depth_msg2->encoding != "32FC1") ||
        (image_depth_msg1->encoding != "16UC1" && image_depth_msg1->encoding != "32FC1") ||
        (image_depth_msg3->encoding != "16UC1" && image_depth_msg3->encoding != "32FC1") ) {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Set "check_subscription_count" to False in the launch file if you want to process it always
    if (!check_subscription_count_ || publisher_pcl->get_subscription_count() > 0)
    {
      // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
      try {
        image_depth_ptr1 = cv_bridge::toCvCopy(
            *image_depth_msg1, 
            sensor_msgs::image_encodings::TYPE_32FC1);
        image_depth_ptr2 = cv_bridge::toCvCopy(
          *image_depth_msg2,
          sensor_msgs::image_encodings::TYPE_32FC1);
        image_depth_ptr3 = cv_bridge::toCvCopy(
          *image_depth_msg3,
          sensor_msgs::image_encodings::TYPE_32FC1);
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      pcl::PointCloud<pcl::PointXYZ> final_pcl, temp_pcl;
      final_pcl.reserve(image_depth_ptr1->image.rows * image_depth_ptr1->image.cols * N_CAMS);
      
      depth2pcl(final_pcl);
      
      // depth2pcl(image_depth_ptr2->image, camera_model2_, temp_pcl);
      // Eigen::Vector3f translation(-1.0, 0.0, -1.0);
      // y_rotation(temp_pcl, final_pcl, translation, -M_PI / 2.0);
      // temp_pcl.clear();


      // depth2pcl(image_depth_ptr3->image, camera_model3_, temp_pcl);
      // translation = Eigen::Vector3f(0.0, 0.0, -2.0);
      // y_rotation(temp_pcl, final_pcl, translation, M_PI);

      sensor_msgs::msg::PointCloud2 out_pointcloud;
      pcl::toROSMsg(final_pcl, out_pointcloud);
      out_pointcloud.header = image_depth_msg1->header;

      // Publish the data
      publisher_pcl->publish(out_pointcloud);
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy1;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy1>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_depth1_, subscription_depth2_, subscription_depth3_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info1_, subscription_info2_, subscription_info3_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcl;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model1_, camera_model2_, camera_model3_;
  cv_bridge::CvImagePtr image_depth_ptr1, image_depth_ptr2, image_depth_ptr3;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_