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

int main(int argc, char** argv) {
  cv::Mat srcImg = cv::imread(argv[1]);
  cv::Mat redChannel;
  cv::namedWindow("【原图】", cv::WINDOW_NORMAL);
  cv::imshow("【原图】", srcImg);
  cv::Mat grayImg;
  std::vector<cv::Mat> channels;
  cv::split(srcImg, channels);
  grayImg = channels.at(0);
  redChannel = channels.at(2);
  cv::namedWindow("【灰度图】", cv::WINDOW_NORMAL);
  cv::imshow("【灰度图】", grayImg);
  // 均值滤波
  cv::blur(grayImg, grayImg, cv::Size(20, 20), cv::Point(-1, -1));
  cv::namedWindow("【均值滤波后】", cv::WINDOW_NORMAL);
  cv::imshow("【均值滤波后】", grayImg);
  // 转化为二值图
  cv::Mat midImg1 = grayImg.clone();
  int rowNumber = midImg1.rows;
  int colNumber = midImg1.cols;

  for (int i = 0; i < rowNumber; i++) {
    uchar* data = midImg1.ptr<uchar>(i);  // 取第i行的首地址
    for (int j = 0; j < colNumber; j++) {
      if (data[j] > 150)
        data[j] = 255;
      else
        data[j] = 0;
    }
  }
  cv::namedWindow("【二值图】", cv::WINDOW_NORMAL);
  cv::imshow("【二值图】", midImg1);
  cv::waitKey(0);

  depth_filter::SegmentLocationFinder finder;

  std::vector<cv::Rect> boundary_boxes;
  finder.FindLocation(midImg1, &boundary_boxes);

  return 0;
}
