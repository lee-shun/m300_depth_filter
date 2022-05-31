/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: feature_matcher.cc
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

#include "pose_correction/modules/feature_matcher.h"

namespace pose_correction {
namespace modules {

void FeatureMatcher::CullWithHistConsistency(
    const std::vector<cv::KeyPoint>& keypoints_ref,
    const std::vector<cv::KeyPoint>& keypoints_cur,
    const std::vector<cv::DMatch>& raw_matches,
    std::vector<cv::DMatch>& good_matches) {
  const int HISTO_LENGTH = 30;
  std::vector<cv::DMatch> rot_hist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; ++i) {
    rot_hist[i].reserve(500);
  }
  const float factor = HISTO_LENGTH / 360.0f;

  for (cv::DMatch match : raw_matches) {
    float angle = keypoints_ref[match.queryIdx].angle -
                  keypoints_cur[match.trainIdx].angle;
    if (angle < 0.0f) {
      angle += 360.f;
    }
    int bin = std::round(angle * factor);
    if (bin == HISTO_LENGTH) {
      bin = 0;
    }

    rot_hist[bin].push_back(match);
  }

  // find the most number in histogram
  size_t max_index = 0;
  size_t max_num = 0;
  for (int i = 0; i < HISTO_LENGTH; ++i) {
    if (rot_hist[i].size() > max_num) {
      max_num = rot_hist[i].size();
      max_index = i;
    }
  }

  good_matches = rot_hist[max_index];
}
}  // namespace modules
}  // namespace pose_correction
