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
  std::vector<std::string> img_names;
  depth_filter::Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");
  dataset.GetAllRGBImageNames(&img_names);
  depth_filter::SegmentLocationFinder finder;

  for (auto img : img_names) {
    cv::Mat binary = cv::imread(img, cv::IMREAD_GRAYSCALE);
    std::vector<cv::Rect> boundary_boxes;
    finder.FindLocation(binary, &boundary_boxes, false);
  }

  return 0;
}
