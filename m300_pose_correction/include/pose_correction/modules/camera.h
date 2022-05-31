/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: camera.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-30
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace pose_correction {
namespace modules {
class Camera {
 public:
  typedef std::shared_ptr<Camera> Ptr;

  explicit Camera(const std::string camera_config_file)
      : file_path_(camera_config_file) {
    std::ifstream fin(file_path_);
    if (!fin) {
      printf("can not open %s in given path, no such file or directory!",
             (file_path_).c_str());
    }

    char camera_name[3];
    for (int k = 0; k < 3; ++k) {
      fin >> camera_name[k];
    }

    double projection_data[12];
    for (int k = 0; k < 12; ++k) {
      fin >> projection_data[k];
    }

    Eigen::Matrix3d K;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    cv::eigen2cv(K, K_);

    Eigen::Vector3d t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;

    fin.close();
  }

 public:
  std::string file_path_;
  cv::Mat K_;
};
}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_
