/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_

#include <Eigen/Core>

#include <mutex>
#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

namespace pose_correction {
namespace modules {
class Frame {
 public:
  typedef std::shared_ptr<Frame> Ptr;
  Frame() {}

  Frame(const uint64_t id, const cv::Mat& img, const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t)
      : img_(img), id_(id), R_(R), t_(t) {}

  /**
   * 创建空的frame, 分配id
   * */
  static Frame::Ptr CreateFrame() {
    static uint64_t factory_id;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
  }

  /**
   * 提取特征点, 计算描述子
   * */
  bool DetectFeatures();

  void GetRT(Eigen::Matrix3d& R, Eigen::Vector3d& t) {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    R = R_;
    t = t_;
  }

  void SetRT(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    R_ = R;
    t_ = t;
  }

  cv::Mat img_;
  uint64_t id_;

  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

 private:
  std::mutex pose_mutex_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d t_;
};

}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_
