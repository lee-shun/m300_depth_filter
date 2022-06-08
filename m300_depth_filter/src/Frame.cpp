/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: Frame.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-07
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/Frame.hpp"
#include "m300_depth_filter/SegmentLocationFinder.hpp"

namespace depth_filter {
bool Frame::DetectFeatures() {
  cv::Mat binary_seg;
  cv::threshold(rgb_seg_, binary_seg, 250, 255, cv::THRESH_BINARY);

  std::vector<cv::Rect> boundary_boxes, patch_boxes;
  std::vector<cv::Point2f> boundary_centers;
  depth_filter::SegmentLocationFinder finder;
  if (!finder.FindLocation(binary_seg, &boundary_boxes, 5, false, false)) {
    return false;
  }

  for (cv::Rect& box : boundary_boxes) {
    float cx = box.x + box.width / 2.0f;
    float cy = box.y + box.height / 2.0f;
    boundary_centers.push_back(cv::Point2f(cx, cy));

    cv::Point2f tl = cv::Point2f(fmax(0, cx - 0.5 * patch_width_),
                                 fmax(0, cy - 0.5 * patch_height_));
    cv::Point2f br =
        cv::Point2f(fmin(binary_seg.cols, cx + 0.5 * patch_width_),
                    fmin(binary_seg.rows, cy + 0.5 * patch_height_));

    patch_boxes.push_back(cv::Rect(tl, br));
  }

  // detect in patch boxes
  cv::Mat patch_mask = cv::Mat::zeros(rgb_img_.size(), CV_8UC1);
  for (auto rect : patch_boxes) {
    patch_mask(rect).setTo(255);
  }

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(rgb_img_, kps_, patch_mask);
  detector->compute(rgb_img_, kps_, descriptor_);

  if (kps_.empty()) return false;

  return true;
}
}  // namespace depth_filter
