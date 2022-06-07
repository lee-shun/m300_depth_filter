/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: dataset.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-31
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_
#define M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace depth_filter {
class Dataset {
 public:
  typedef std::shared_ptr<Dataset> Ptr;

  explicit Dataset(const std::string dataset_path)
      : dataset_path_(dataset_path) {
    local_pose_path_ = dataset_path_ + "/local_pose.csv";
    rgb_img_path_ = dataset_path_ + "/rgb";
  }

  bool GetAllRGBImageNames(std::vector<std::string>* all_image_names,
                        const bool use_cv_global = false);

  /**
   * NOTE: the index is same as the images' index
   * */
  bool ReadLocalPose(const std::string filename, const int frame_index,
                           Eigen::Vector3d* trans);

 private:
  std::string dataset_path_;
  std::string local_pose_path_;
  std::string rgb_img_path_;
};
}  // namespace depth_filter

#endif  // M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_
