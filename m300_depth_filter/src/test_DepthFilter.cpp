/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_DepthFilter.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-27
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/DepthFilter.hpp"
#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/SystemLib.hpp"

bool ReadPose(const std::string filename, const int line_index,
              int* frame_index, Sophus::SE3d* Twc) {
  std::ifstream fin;
  fin.open(filename);
  if (!fin) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                filename.c_str());
    return false;
  }
  std::string pose_tmp;
  std::vector<double> pose_elements;
  depth_filter::SeekToLine(fin, line_index);
  // read each index, x, y, z, everytime
  for (int i = 0; i < 13; ++i) {
    if (!getline(fin, pose_tmp, ',')) {
      PRINT_ERROR("pose reading error! at line_index %d", line_index);
      return false;
    }
    // PRINT_DEBUG("read trans:index+xyz:%.8f", std::stod(trans_tmp));
    pose_elements.push_back(std::stod(pose_tmp));
  }

  (*frame_index) = pose_elements[0];

  Eigen::Vector3d t;
  t << pose_elements[4], pose_elements[8], pose_elements[12];

  Eigen::Matrix3d R;
  R << pose_elements[1], pose_elements[2], pose_elements[3], pose_elements[5],
      pose_elements[6], pose_elements[7], pose_elements[9], pose_elements[10],
      pose_elements[11];
  // normalize it
  Eigen::Quaterniond q(R);
  q.normalize();

  (*Twc) = Sophus::SE3d(q, t);

  return true;
}

int main(int argc, char** argv) {
  const std::string home = std::getenv("HOME");
  const std::string dataset_path =
      home + "/m300_depth/m300_grabbed_data_1_17.1";
  const std::string translation_path = dataset_path + "/local_pose.csv";
  const std::string img_path = dataset_path + "/rgb";

  depth_filter::DepthFilter filter;
  depth_filter::DepthFilter::Param param;

  // STEP: read the ref image
  cv::Mat ref_img = cv::imread(img_path + "/1.png", 0);
  double init_depth = 10.0;  // 深度初始值
  double init_cov2 = 10.0;   // 方差初始值
  cv::Mat depth(param.height, param.width, CV_64F, init_depth);  // 深度图
  cv::Mat depth_cov2(param.height, param.width, CV_64F,
                     init_cov2);  // 深度图方差

  // TODO: 指定ref_point
  Eigen::Vector2d ref_point(707, 386);

  // STEP: read the ref translation
  Sophus::SE3d TWR;

  // read updates from index(image name: 1)
  for (int i = 2; i < 10; ++i) {
    cv::Mat cur_img =
        cv::imread(img_path + "/" + std::to_string(i) + ".png", 0);
    Eigen::Vector3d cur_trans;
    Sophus::SE3d TWC(Eigen::Matrix3d::Identity(), cur_trans);

    Sophus::SE3d TCR = TWC.inverse() * TWR;

    std::cout << "TCR.rotation" << TCR.rotationMatrix() << std::endl;
    std::cout << "TCR.translation" << TCR.translation() << std::endl;

    if (ref_img.empty() || cur_img.empty()) {
      PRINT_ERROR("image index: %d is Empty!", i);
      return 1;
    }

    filter.UpdateDepth(ref_img, cur_img, TCR, ref_point, depth, depth_cov2);
  }

  // 输出最后的结果
  cv::imshow("depth_estimation", depth * 0.2);
  cv::waitKey(0);
  PRINT_INFO("depth: %f", depth.ptr<double>(707)[386]);
  return 0;
}
