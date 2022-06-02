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

void FeatureMatcher::MacthFeaturesBF(const modules::Frame::Ptr ref_frame,
                                     const modules::Frame::Ptr cur_frame,
                                     std::vector<cv::Point2f>& match_pts_ref,
                                     std::vector<cv::Point2f>& match_pts_cur,
                                     bool show_matches) {
  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(ref_frame->descriptors_, cur_frame->descriptors_, matches);

  std::vector<cv::DMatch> good_matches, refined_hist_matches;

  // distance
  for (cv::DMatch& match : matches) {
    if (match.distance <= 50) good_matches.push_back(match);
  }

  CullWithHistConsistency(ref_frame->kps_, cur_frame->kps_, good_matches,
                          refined_hist_matches);

  // fill all the refined matches
  for (cv::DMatch& m : refined_hist_matches) {
    match_pts_ref.push_back(ref_frame->kps_pt_[m.queryIdx]);
    match_pts_cur.push_back(cur_frame->kps_pt_[m.trainIdx]);
  }

  if (show_matches) {
    cv::Mat img_good_match;
    cv::drawMatches(ref_frame->img_, ref_frame->kps_, cur_frame->img_,
                    cur_frame->kps_, refined_hist_matches, img_good_match);
    cv::imshow("matches", img_good_match);
    cv::waitKey(0);
  }
}

void FeatureMatcher::CullWithHistConsistency(
    const std::vector<cv::KeyPoint>& keypoints_ref,
    const std::vector<cv::KeyPoint>& keypoints_cur,
    const std::vector<cv::DMatch>& raw_matches,
    std::vector<cv::DMatch>& good_matches) {
  const int HISTO_LENGTH = 60;
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
