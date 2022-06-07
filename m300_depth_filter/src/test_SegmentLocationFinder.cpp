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

#include "m300_depth_filter/SegmentLocationFinder.hpp"
#include "m300_depth_filter/Dataset.hpp"

int main(int argc, char** argv) {
  std::vector<std::string> rgb_img_names, mask_img_names;
  depth_filter::Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");
  dataset.GetAllRGBImageNames(&rgb_img_names);
  dataset.GetAllMaskImageNames(&mask_img_names);
  depth_filter::SegmentLocationFinder finder;

  for (size_t i = 0; i < rgb_img_names.size(); ++i) {
    cv::Mat mask_img = cv::imread(mask_img_names[i], cv::IMREAD_GRAYSCALE);
    cv::Mat mask_binary;
    cv::threshold(mask_img, mask_binary, 250, 255, cv::THRESH_BINARY);

    std::vector<cv::Rect> boundary_boxes;
    finder.FindLocation(mask_binary, &boundary_boxes, 5, false, true);

    cv::Mat rgb_img = cv::imread(rgb_img_names[i]);
    for (cv::Rect& box : boundary_boxes) {
      cv::rectangle(rgb_img, box, cv::Scalar(0, 255, 0), 2);
    }
    cv::imshow("rgb", rgb_img);
    cv::waitKey(0);
  }

  return 0;
}
