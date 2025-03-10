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

namespace computer_vision
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using namespace std::chrono_literals;

int N_CAMS = 5;

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

    camera_model1_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model1_->fromCameraInfo(get_instrinc_cam1());

    camera_model2_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model2_->fromCameraInfo(get_instrinc_cam2());

    camera_model3_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model3_->fromCameraInfo(get_instrinc_cam3());

    camera_model4_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model4_->fromCameraInfo(get_instrinc_cam4());

    camera_model5_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model5_->fromCameraInfo(get_instrinc_cam5());

    subscription_depth1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in1", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in2", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth3_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in3", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth4_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in4", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    subscription_depth5_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in5", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(1000000), *subscription_depth1_, *subscription_depth2_, *subscription_depth3_, *subscription_depth4_, *subscription_depth5_);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2, _3, _4, _5));

    publisher_pcl = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_sync",
      rclcpp::SensorDataQoS().reliable());

    tick = std::chrono::high_resolution_clock::now();
  }

private:
  bool check_subscription_count_;

  sensor_msgs::msg::CameraInfo get_instrinc_cam1(){
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Rellenar el header
    camera_info_msg.header.stamp.sec = 1741618763;
    camera_info_msg.header.stamp.nanosec = 592913061;
    camera_info_msg.header.frame_id = "oak1_right_camera_optical_frame";

    // Dimensiones de la imagen
    camera_info_msg.height = 400;
    camera_info_msg.width = 640;

    // Modelo de distorsión
    camera_info_msg.distortion_model = "rational_polynomial";
    camera_info_msg.d = {-0.8970510363578796, -3.311840776228784, 
                         -0.001362776754446628, -0.00056734721874818121, 
                          3.88497710227963, -3.9050838947296143, 
                         -3.292762728305037,  3.8748250007629395};

    // Matriz intrínseca K
    camera_info_msg.k = {454.27581787109375,  0.0, 305.753994140625,
                         0.0,  454.132960878906, 189.11952209472656,
                         0.0,  0.0,  1.0};

    // Matriz de rotación R
    camera_info_msg.r = {0.9999306201934814, -2.8355399722027e-05,  0.0117860680500735,
                         4.842148880497356e-05,  0.9999985694885254, -0.0010730232120353,
                         -0.011780061249137938,  0.0010734754855558276, 0.9999291300773621};

    // Matriz de proyección P
    camera_info_msg.p = {454.27581787109375,  0.0, 305.753994140625, 0.0,
                         0.0,  454.132960878906, 189.11952209472656, 0.0,
                         0.0,  0.0,  1.0,  0.0};

    // Binning y ROI
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;
    camera_info_msg.roi.x_offset = 0;
    camera_info_msg.roi.y_offset = 0;
    camera_info_msg.roi.height = 0;
    camera_info_msg.roi.width = 0;
    camera_info_msg.roi.do_rectify = false;
    
    return camera_info_msg;
  }

  sensor_msgs::msg::CameraInfo get_instrinc_cam2(){
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Rellenar el header
    camera_info_msg.header.stamp.sec = 1741619400;
    camera_info_msg.header.stamp.nanosec = 311082180;
    camera_info_msg.header.frame_id = "oak2_right_camera_optical_frame";

    // Dimensiones de la imagen
    camera_info_msg.height = 400;
    camera_info_msg.width = 640;

    // Modelo de distorsión
    camera_info_msg.distortion_model = "rational_polynomial";
    camera_info_msg.d = {-0.1039981096982956, -4.7574286460876465, 
                         -0.002679175373046627, -0.000963595924262285, 
                          4.5111036643992676, -0.1461227387189865, 
                          0.6510576671651001, -4.44262790679316};

    // Matriz intrínseca K
    camera_info_msg.k = {451.0455947265625,  0.0, 328.9924621582031,
                         0.0,  450.8662109375, 235.92983232421875,
                         0.0,  0.0,  1.0};

    // Matriz de rotación R
    camera_info_msg.r = {0.999999996986389, -0.004149611108005047, -0.0008748080898603697,
                         0.004159019345578495,  0.9999552369117737, -0.008302662818532448,
                         0.008394789765589839,  0.008506221696734428, 0.9999634623527527};

    // Matriz de proyección P
    camera_info_msg.p = {451.0455947265625,  0.0, 328.9924621582031, 0.0,
                         0.0,  450.8662109375, 235.92983232421875, 0.0,
                         0.0,  0.0,  1.0,  0.0};

    // Binning y ROI
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;
    camera_info_msg.roi.x_offset = 0;
    camera_info_msg.roi.y_offset = 0;
    camera_info_msg.roi.height = 0;
    camera_info_msg.roi.width = 0;
    camera_info_msg.roi.do_rectify = false;
    
    return camera_info_msg;
  }

  sensor_msgs::msg::CameraInfo get_instrinc_cam3(){
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Rellenar el header
    camera_info_msg.header.stamp.sec = 1741619512;
    camera_info_msg.header.stamp.nanosec = 400881677;
    camera_info_msg.header.frame_id = "oak3_right_camera_optical_frame";

    // Dimensiones de la imagen
    camera_info_msg.height = 400;
    camera_info_msg.width = 640;

    // Modelo de distorsión
    camera_info_msg.distortion_model = "rational_polynomial";
    camera_info_msg.d = {-1.2734373807907104, -1.9612691402435303, 
                         -0.00182633326492106559, 0.00102855222441256, 
                          2.6471283735031562, -1.2939773797988892, 
                         -1.9068445540410362, -2.611210823059082};

    // Matriz intrínseca K
    camera_info_msg.k = {454.3362121582031,  0.0, 305.989990234375,
                         0.0,  454.31610107421875, 204.17007446289062,
                         0.0,  0.0,  1.0};

    // Matriz de rotación R
    camera_info_msg.r = {0.9999585151672363,  0.00862814480602764, -0.00291972677233844,
                         -0.00862946268177072,  0.9999628742984706, -0.000493437317011512,
                         0.0029158205629203653,  0.000446209180790931, 0.9999956488609314};

    // Matriz de proyección P
    camera_info_msg.p = {454.3362121582031,  0.0, 305.989990234375, 0.0,
                         0.0,  454.31610107421875, 204.17007446289062, 0.0,
                         0.0,  0.0,  1.0,  0.0};

    // Binning y ROI
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;
    camera_info_msg.roi.x_offset = 0;
    camera_info_msg.roi.y_offset = 0;
    camera_info_msg.roi.height = 0;
    camera_info_msg.roi.width = 0;
    camera_info_msg.roi.do_rectify = false;
    
    return camera_info_msg;
  }

  sensor_msgs::msg::CameraInfo get_instrinc_cam4(){
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Header
    camera_info_msg.header.stamp.sec = 1741619583;
    camera_info_msg.header.stamp.nanosec = 898378322;
    camera_info_msg.header.frame_id = "oak4_right_camera_optical_frame";

    // Image dimensions
    camera_info_msg.height = 400;
    camera_info_msg.width = 640;

    // Distortion model
    camera_info_msg.distortion_model = "rational_polynomial";

    // Distortion coefficients (D)
    camera_info_msg.d = {
        11.517766952514648, 
        3.9527781009674072, 
        -0.00048372796739705969, 
        -0.000631666954793036, 
        -19.346494674682617, 
        11.58913344116211, 
        3.578423261642456, 
        -18.894325256347656
    };

    // Camera matrix (K)
    camera_info_msg.k = {
        457.55975341796875, 0.0, 319.53515625,
        0.0, 457.47283935546875, 201.418701171875,
        0.0, 0.0, 1.0
    };

    // Rectification matrix (R)
    camera_info_msg.r = {
        0.9999346137046814, 0.005614885129034519, 0.00962455369352108,
        -0.005592220932519964, 0.9998977180714054, 0.00203136862309518,
        -0.00975194943130273, -0.00224550603888929, 0.999947772762532
    };

    // Projection matrix (P)
    camera_info_msg.p = {
        457.55975341796875, 0.0, 319.53515625, 0.0,
        0.0, 457.47283935546875, 201.418701171875, 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    // Binning
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;

    // Region of Interest (ROI)
    camera_info_msg.roi.x_offset = 0;
    camera_info_msg.roi.y_offset = 0;
    camera_info_msg.roi.height = 0;
    camera_info_msg.roi.width = 0;
    camera_info_msg.roi.do_rectify = false;
    
    return camera_info_msg;
  }

  sensor_msgs::msg::CameraInfo get_instrinc_cam5(){
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Header
    camera_info_msg.header.stamp.sec = 1741619776;
    camera_info_msg.header.stamp.nanosec = 682764490;
    camera_info_msg.header.frame_id = "oak5_right_camera_optical_frame";

    // Image dimensions
    camera_info_msg.height = 400;
    camera_info_msg.width = 640;

    // Distortion model
    camera_info_msg.distortion_model = "rational_polynomial";

    // Distortion coefficients (D)
    camera_info_msg.d = {
        -1.0620687007904053, 
        -2.320189997106254, 
        0.0006696623167954385, 
        0.0017430468393272131, 
        2.789074659347534, 
        -1.088186264038086, 
        -2.2531087398529053, 
        2.745926147388916
    };

    // Camera matrix (K)
    camera_info_msg.k = {
        450.6310729980469, 0.0, 309.54864501953125,
        0.0, 450.7037048339844, 199.56655883789062,
        0.0, 0.0, 1.0
    };

    // Rectification matrix (R)
    camera_info_msg.r = {
        0.9999790191650391, -0.001678782779257955, 0.006021526884446096,
        0.0018920677034557808, 0.9999883177224121, 0.004898851217187405,
        -0.006129822982213247, -0.00491520743578672, 0.9999867671661377
    };

    // Projection matrix (P)
    camera_info_msg.p = {
        450.6310729980469, 0.0, 309.54864501953125, 0.0,
        0.0, 450.7037048339844, 199.56655883789062, 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    // Binning
    camera_info_msg.binning_x = 0;
    camera_info_msg.binning_y = 0;

    // Region of Interest (ROI)
    camera_info_msg.roi.x_offset = 0;
    camera_info_msg.roi.y_offset = 0;
    camera_info_msg.roi.height = 0;
    camera_info_msg.roi.width = 0;
    camera_info_msg.roi.do_rectify = false;
    
    return camera_info_msg;
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
      const float* ptr = (float*)input.ptr<uint16_t>(row);
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
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg2,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg3,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg4,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg5)
  {
    // Check if depth image has been received
    if ((image_depth_msg2->encoding != "16UC1" && image_depth_msg2->encoding != "32FC1") ||
        (image_depth_msg1->encoding != "16UC1" && image_depth_msg1->encoding != "32FC1") ||
        (image_depth_msg3->encoding != "16UC1" && image_depth_msg3->encoding != "32FC1") ||
        (image_depth_msg4->encoding != "16UC1" && image_depth_msg4->encoding != "32FC1") ||
        (image_depth_msg5->encoding != "16UC1" && image_depth_msg5->encoding != "32FC1")) {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Set "check_subscription_count" to False in the launch file if you want to process it always
    if (!check_subscription_count_ || publisher_pcl->get_subscription_count() > 0)
    {
      // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
      cv_bridge::CvImagePtr image_depth_ptr1, image_depth_ptr2, image_depth_ptr3, image_depth_ptr4, image_depth_ptr5;
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
        image_depth_ptr4 = cv_bridge::toCvCopy(
          *image_depth_msg4,
          sensor_msgs::image_encodings::TYPE_32FC1);
        image_depth_ptr5 = cv_bridge::toCvCopy(
          *image_depth_msg5,
          sensor_msgs::image_encodings::TYPE_32FC1);
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      pcl::PointCloud<pcl::PointXYZ> final_pcl, temp_pcl;
      final_pcl.reserve(image_depth_ptr1->image.rows * image_depth_ptr1->image.cols * N_CAMS);
      
      depth2pcl(image_depth_ptr1->image, camera_model1_, final_pcl);
      Eigen::Vector3f translation2(0.0, 0.0, 0);
      Eigen::Vector3f translation(-0.075, 0.0, -0.04330);

      
      // // Crear la matriz de transformación (rotación en el eje y)
      depth2pcl(image_depth_ptr2->image, camera_model2_, temp_pcl);
      Eigen::Affine3f transform;
      // correction with icp + respective rototraslation
      transform.matrix() <<   0.5392422,  -0.08248261, -0.88427424, -0.10105343,
                              -0.0116933,   0.995033,   -0.0988921,  -0.00625785,
                              0.8420733,   0.05579372,  0.45638803, -0.00985628,
                              0.,          0.,          0.,          1.;        
         
      pcl::transformPointCloud(temp_pcl, temp_pcl, transform);
      final_pcl.insert(final_pcl.end(), temp_pcl.begin(), temp_pcl.end());

      temp_pcl.clear();

      depth2pcl(image_depth_ptr3->image, camera_model3_, temp_pcl);
      // correction with icp + respective rototraslation
      transform.matrix() <<    5.0052887e-01,  2.6675247e-02,  8.6531043e-01,  7.3455721e-02,
                              -8.0717099e-04,  9.9953997e-01, -3.0346701e-02, -1.0415400e-02,
                              -8.6572003e-01,  1.4490918e-02,  5.0032032e-01, -6.7022793e-02,
                               0.0,  0.0,  0.0,  1.0;        
         
      pcl::transformPointCloud(temp_pcl, temp_pcl, transform);
      final_pcl.insert(final_pcl.end(), temp_pcl.begin(), temp_pcl.end());

      temp_pcl.clear();

      depth2pcl(image_depth_ptr4->image, camera_model4_, temp_pcl);
      // correction with icp + respective rototraslation
      transform.matrix() <<    -4.8677036e-01,  6.5628223e-02, -8.7106133e-01, -1.3400421e-01,
                               2.6426499e-04,  9.9718499e-01,  7.4983001e-02,  4.8022801e-03,
                               8.7352961e-01,  3.6269389e-02, -4.8541749e-01, -8.8113561e-02,
                               0.0,  0.0,  0.0,  1.0;       
         
      pcl::transformPointCloud(temp_pcl, temp_pcl, transform);
      final_pcl.insert(final_pcl.end(), temp_pcl.begin(), temp_pcl.end());

      temp_pcl.clear();

      depth2pcl(image_depth_ptr5->image, camera_model5_, temp_pcl);
      // correction with icp + respective rototraslation
      transform.matrix() <<    -4.9431553e-01, -7.5284652e-03,  8.6925077e-01,  1.8517042e-02,
                              -8.5253699e-04,  9.9996603e-01,  8.1758099e-03,  4.0410701e-03,
                              -8.6928308e-01,  3.3003348e-03, -4.9430552e-01, -1.5657477e-01,
                               0.0,  0.0,  0.0,  1.0;       
         
      pcl::transformPointCloud(temp_pcl, temp_pcl, transform);
      final_pcl.insert(final_pcl.end(), temp_pcl.begin(), temp_pcl.end());

      // final icp correction pcl_sync to lidar
      // correction with icp + respective rototraslation
      transform.matrix() <<    0.999147, -0.0152454,  0.0384261,  -0.144349,
                              0.00724112,  0.979687,  0.200407,  0.0892053,
                              -0.0407008,  -0.199957, 0.978961, -0.188059,
                               0.0,  0.0,  0.0,  1.0;       
         
      pcl::transformPointCloud(final_pcl, final_pcl, transform);

      sensor_msgs::msg::PointCloud2 out_pointcloud;
      pcl::toROSMsg(final_pcl, out_pointcloud);
      out_pointcloud.header = image_depth_msg1->header;

      auto tock = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = tock - tick;
      if (duration.count() >= 0.099){
        publisher_pcl->publish(out_pointcloud);
        tick = tock;
      }
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_depth1_, subscription_depth2_, subscription_depth3_, subscription_depth4_, subscription_depth5_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcl;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model1_, camera_model2_, camera_model3_, camera_model4_, camera_model5_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::high_resolution_clock> tick;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__DEPTHSYNC_HPP_