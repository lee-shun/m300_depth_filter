/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: SegmentLocationFinder.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-06
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/SegmentLocationFinder.hpp"
#include "m300_depth_filter/PrintCtrlMacro.h"
#include <opencv2/imgproc.hpp>

bool depth_filter::SegmentLocationFinder::FindLocation(
    const cv::Mat binary_input, std::vector<cv::Rect>* boundary_box,
    const int morph_size, const bool imshow_contours,
    const bool imshow_final_rect) {
  cv::Mat binary = binary_input.clone();

  cv::Mat show_img;
  cv::cvtColor(binary, show_img, cv::COLOR_GRAY2BGR);

  // STEP: 1 morphology
  if (morph_size != 0) {
    cv::Mat after_open_binary;
    cv::Mat after_close_binary;

    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_size, morph_size));

    cv::morphologyEx(binary, after_open_binary, cv::MORPH_OPEN, element);
    cv::morphologyEx(after_open_binary, after_close_binary, cv::MORPH_CLOSE,
                     element);

    binary = after_close_binary;
  }

  // STEP: 2 find contours
  cv::Mat contours_img = cv::Mat::zeros(binary.rows, binary.cols, CV_8UC3);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    PRINT_WARN("no contours found!");
    return false;
  }

  if (imshow_contours) {
    int index = 0;
    for (; index >= 0; index = hierarchy[index][0]) {
      cv::Scalar color(255, 255, 255);
      drawContours(contours_img, contours, index, color, 0, 8, hierarchy);
    }

    cv::namedWindow("contours image:", cv::WINDOW_NORMAL);
    cv::imshow("contours image:", contours_img);
    cv::waitKey(0);
  }

  // STEP: 3 find rectangles
  for (int i = 0; i < contours.size(); i++) {
    std::vector<cv::Point> points = contours[i];
    cv::Rect box = cv::boundingRect(cv::Mat(points));
    boundary_box->push_back(box);

    if (imshow_final_rect) {
      cv::Point center;
      center.x = box.x + box.width / 2.0f;
      center.y = box.y + box.height / 2.0f;
      // draw rects
      cv::rectangle(show_img, box, cv::Scalar(0, 255, 0), 2);

      // center
      cv::Point l, r, u, d;
      l.x = center.x - 10;
      l.y = center.y;

      r.x = center.x + 10;
      r.y = center.y;

      u.x = center.x;
      u.y = center.y - 10;

      d.x = center.x;
      d.y = center.y + 10;
      cv::line(show_img, l, r, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(show_img, u, d, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
  }

  if (imshow_final_rect) {
    cv::namedWindow("final rect", cv::WINDOW_NORMAL);
    cv::imshow("final rect", show_img);
    cv::waitKey(0);
  }

  return true;
}
