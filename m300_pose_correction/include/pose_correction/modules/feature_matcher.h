/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: feature_matcher.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace pose_correction {
namespace modules {
class FeatureMatcher {
 public:
  FeatureMatcher() {}

  void CullWithHistConsistency(const std::vector<cv::KeyPoint>& keypoints_ref,
                               const std::vector<cv::KeyPoint>& keypoints_cur,
                               const std::vector<cv::DMatch>& raw_matches,
                               std::vector<cv::DMatch>& good_matches);
};
}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_
