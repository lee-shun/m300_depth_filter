/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_SegmentLocationFinder.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-16
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/SegmentLocationFinder.hpp"
#include "m300_depth_filter/Dataset.hpp"

#include <opencv2/features2d.hpp>

#include <cmath>

const float patch_width = 90.0f;
const float patch_height = 90.0f;

int main(int argc, char** argv) {
  std::vector<std::string> rgb_img_names, mask_img_names;
  depth_filter::Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");
  dataset.GetAllRGBImageNames(&rgb_img_names);
  dataset.GetAllMaskImageNames(&mask_img_names);
  depth_filter::SegmentLocationFinder finder;

  // for each of the images
  cv::Mat ref_img, ref_desc;
  std::vector<cv::KeyPoint> ref_kps;
  for (size_t i = 0; i < rgb_img_names.size(); ++i) {
    PRINT_DEBUG("Frame ID: %zu", i);
    cv::Mat mask_img = cv::imread(mask_img_names[i], cv::IMREAD_GRAYSCALE);
    cv::Mat mask_binary;
    cv::threshold(mask_img, mask_binary, 250, 255, cv::THRESH_BINARY);

    std::vector<cv::Rect> boundary_boxes, patch_boxes;
    std::vector<cv::Point2f> boundary_centers;
    if (!finder.FindLocation(mask_binary, &boundary_boxes, 5, false, true))
      continue;

    cv::Mat rgb_img = cv::imread(rgb_img_names[i], cv::IMREAD_GRAYSCALE);
    cv::Mat rgb_show1 = rgb_img.clone();

    for (cv::Rect& box : boundary_boxes) {
      float cx = box.x + box.width / 2.0f;
      float cy = box.y + box.height / 2.0f;
      boundary_centers.push_back(cv::Point2f(cx, cy));

      cv::Point2f tl = cv::Point2f(fmax(0, cx - 0.5 * patch_width),
                                   fmax(0, cy - 0.5 * patch_height));
      cv::Point2f br = cv::Point2f(fmin(rgb_img.cols, cx + 0.5 * patch_width),
                                   fmin(rgb_img.rows, cy + 0.5 * patch_height));

      patch_boxes.push_back(cv::Rect(tl, br));
      cv::rectangle(rgb_show1, cv::Rect(tl, br), cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow("rgb", rgb_show1);

    // STEP: 在patch boxes 中提取特征点

    cv::Mat patch_mask = cv::Mat::zeros(rgb_img.size(), CV_8UC1);
    for (auto rect : patch_boxes) {
      patch_mask(rect).setTo(255);
    }

    cv::imshow("patch", patch_mask);

    // 提取orb角点
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> kps;
    detector->detect(rgb_img, kps, patch_mask);
    cv::Mat rgb_with_kps = rgb_img.clone();
    cv::drawKeypoints(rgb_img, kps, rgb_with_kps);
    cv::imshow("kps", rgb_with_kps);

    cv::Mat descriptors;
    detector->compute(rgb_img, kps, descriptors);

    if (i == 23) {
      ref_img = rgb_img;
      ref_desc = descriptors;
      ref_kps = kps;
    }

    if (i >= 23) {
      // match
      std::vector<cv::DMatch> matches;
      cv::BFMatcher matcher(cv::NORM_HAMMING);
      matcher.match(ref_desc, descriptors, matches);
      std::vector<cv::DMatch> good_matches;

      // distance
      for (cv::DMatch& match : matches) {
        if (match.distance <= 30) good_matches.push_back(match);
      }

      cv::Mat img_good_match;

      cv::drawMatches(ref_img, ref_kps, rgb_img, kps, good_matches,
                      img_good_match);

      cv::imshow("matches", img_good_match);
    }

    cv::waitKey(0);
  }

  return 0;
}
