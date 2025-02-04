/*
# Copyright (c) 2025 Adrián Cobo Merino
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <image_geometry/pinhole_camera_model.hpp>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <omp.h>

using namespace std::chrono_literals;

geometry_msgs::msg::TransformStamped camera2basefootprint;

class ComputerVisionSubscriber : public rclcpp::Node
{
public:
  ComputerVisionSubscriber()
  : Node("opencv_subscriber")
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 100));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    subscription_dist_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth_input", qos,
      std::bind(
        &ComputerVisionSubscriber::distance_image_callback, this, std::placeholders::_1));

    subscription_camera_intrinsic_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/depth_camera_info", qos,
      std::bind(
        &ComputerVisionSubscriber::intrinsic_params_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcl_output",
      rclcpp::SensorDataQoS().reliable());

    timer_ = create_wall_timer(50ms, std::bind(&ComputerVisionSubscriber::on_timer, this));
    got_depthimg_ = false;
    got_cam_info_ = false;
  }

private:
  void on_timer()
  {
    // Image processing
    if(!got_depthimg_ || !got_cam_info_ || publisher_->get_subscription_count() == 0){return;}
    cv_bridge::CvImagePtr depth_image_ptr =
      cv_bridge::toCvCopy(msg_ptr_, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat image_raw = depth_image_ptr->image;
    cv::Mat resized_img;
    //cv::resize(image_raw, resized_img, cv::Size(480, 640), 0, 0, cv::INTER_LINEAR);
    pcl::PointCloud<pcl::PointXYZ> pcl_out = depth2pcl(image_raw);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 out_pointcloud;
    pcl::toROSMsg(pcl_out, out_pointcloud);
    out_pointcloud.header = msg_ptr_->header;

    // Publish the data
    publisher_->publish(out_pointcloud);
  }

  pcl::PointCloud<pcl::PointXYZ> depth2pcl(cv::Mat input)
  {
    pcl::PointCloud<pcl::PointXYZ> out_pointcloud;
    out_pointcloud.reserve(input.rows * input.cols);

    // Recorrer la imagen fila por fila
    #pragma omp parallel for
    for (int row = 0; row < input.rows; ++row) {
      const float* ptr = input.ptr<float>(row);
      std::vector<pcl::PointXYZ> local_points;  // Cada hilo usa un vector local

      for (int col = 0; col < input.cols; ++col) {
        float d = ptr[col] / 1000.0f;
        if (!std::isfinite(d)) continue;

        float x_3d = (row - cx_) * d / fx_;
        float y_3d = (col - cy_) * d / fy_;
        float z_3d = d;

        local_points.emplace_back(x_3d, y_3d, z_3d);
      }

      #pragma omp critical
      out_pointcloud.insert(out_pointcloud.end(), local_points.begin(), local_points.end());
    }
  
    return out_pointcloud;
  }

  void distance_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if(publisher_->get_subscription_count() == 0){return;}
    // Convertir los datos de profundidad a un objeto Mat de OpenCV
    msg_ptr_ = msg;
    got_depthimg_ = true;
  }

  void intrinsic_params_callback(const sensor_msgs::msg::CameraInfo msg)
  {
    if(publisher_->get_subscription_count() == 0){return;}
    image_geometry::PinholeCameraModel intrinsic_camera_matrix = image_geometry::PinholeCameraModel();
    intrinsic_camera_matrix.fromCameraInfo(msg);
    cv::Mat intrinsic_marix = (cv::Mat)intrinsic_camera_matrix.intrinsicMatrix();

    fx_ = (float)intrinsic_marix.at<double>(0, 0);
    fy_ = (float)intrinsic_marix.at<double>(1, 1);
    cx_ = (float)intrinsic_marix.at<double>(0, 2);
    cy_ = (float)intrinsic_marix.at<double>(1, 2);

    got_cam_info_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_dist_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_intrinsic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  sensor_msgs::msg::Image::SharedPtr msg_ptr_;
  bool got_depthimg_, got_cam_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  float fx_, fy_, cx_, cy_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}