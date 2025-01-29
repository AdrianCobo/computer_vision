/*
  Code adapted from: https://github.com/jmguerreroh/computer_vision/blob/humble/include/computer_vision/CVSubscriber.hpp
  Copyright (c) 2024 José Miguel Guerrero Hernández
  Copyright (c) 2025 Adrián Cobo Merino

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

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

namespace computer_vision
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// TODO: arreglar esto con los argumentos que usas
class CVGroup
{
public:
  CVGroup(cv::Mat image_rgb, cv::Mat image_depth, pcl::PointCloud<pcl::PointXYZRGB> pointcloud)
  {
    image_rgb_ = image_rgb;
    image_depth_ = image_depth;
    pointcloud_ = pointcloud;
  }
  cv::Mat getImageRGB() {return image_rgb_;}
  cv::Mat getImageDepth() {return image_depth_;}
  pcl::PointCloud<pcl::PointXYZRGB> getPointCloud() {return pointcloud_;}

private:
  cv::Mat image_rgb_;
  cv::Mat image_depth_;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_;
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

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(100), *subscription_depth1_, *subscription_depth2_, *subscription_depth3_);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2, _3));

    publisher_depth2_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_depth2",
      rclcpp::SensorDataQoS().reliable());

    publisher_depth1_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_depth1",
      rclcpp::SensorDataQoS().reliable());

    publisher_depth3_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_depth3",
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

  void topic_callback_multi(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg1,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg2,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg3) const
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
    if (image_depth_msg2->encoding != "16UC1" && image_depth_msg2->encoding != "32FC1") {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Set "check_subscription_count" to False in the launch file if you want to process it always
    if (!check_subscription_count_ ||
      ((publisher_depth1_->get_subscription_count() > 0 ) &&
      (publisher_depth2_->get_subscription_count() > 0) &&
      (publisher_depth3_->get_subscription_count() > 0)))
    {
      // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
      cv_bridge::CvImagePtr image_depth_ptr1, image_depth_ptr2, image_depth_ptr3;
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
      cv::Mat image_depth_raw1 = image_depth_ptr1->image;
      cv::Mat image_depth_raw2 = image_depth_ptr2->image;
      cv::Mat image_depth_raw3 = image_depth_ptr3->image;

      // Image and PointCloud processing
      DepthSync cvgroup = CVGroup(image_depth_raw1, image_depth_raw2, image_depth_raw3);

      // Convert OpenCV Image to ROS Image
      cv_bridge::CvImage image_depth_bridge1 =
        cv_bridge::CvImage(
        image_depth_msg1->header, sensor_msgs::image_encodings::TYPE_32FC1,
        cvgroup.getImageDepth1());

      cv_bridge::CvImage image_depth_bridge2 =
        cv_bridge::CvImage(
        image_depth_msg2->header, sensor_msgs::image_encodings::TYPE_32FC1,
        cvgroup.getImageDepth2());

      cv_bridge::CvImage image_depth_bridge3 =
        cv_bridge::CvImage(
        image_depth_msg3->header, sensor_msgs::image_encodings::TYPE_32FC1,
        cvgroup.getImageDepth3());
    

      // >> message to be sent
      sensor_msgs::msg::Image out_image_depth1, out_image_depth2, out_image_depth3;

      // from cv_bridge to sensor_msgs::Image
      image_depth_bridge1.toImageMsg(out_image_depth1);
      image_depth_bridge2.toImageMsg(out_image_depth2);
      image_depth_bridge3.toImageMsg(out_image_depth3);

      // Publish the data
      publisher_depth1_->publish(out_image_depth1);
      publisher_depth2_->publish(out_image_depth2);
      publisher_depth3_->publish(out_image_depth3);
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_depth1_, subscription_depth2_, subscription_depth3_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info1_, subscription_info2_, subscription_info3_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth1_, publisher_depth2_, publisher_depth3_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model1_, camera_model2_, camera_model3_;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_