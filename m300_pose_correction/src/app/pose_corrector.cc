/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: pose_corrector.cc
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

#include "pose_correction/app/pose_corrector.h"
#include <opencv2/core/eigen.hpp>

namespace pose_correction {
namespace app {

void PoseCorrector::Run() {}

void PoseCorrector::EstimatePose(const std::vector<cv::Point2f>& pts_ref,
                                 const std::vector<cv::Point2f>& pts_cur,
                                 const double scale,
                                 const modules::Frame::Ptr ref_frame,
                                 modules::Frame::Ptr cur_frame) {
  cv::Mat E, dR, dt, K;
  cv::eigen2cv(camera_->K_, K);

  E = cv::findEssentialMat(pts_cur, pts_ref, K, cv::RANSAC);
  cv::recoverPose(E, pts_cur, pts_ref, K, dR, dt);

  Sophus::SE3d T_ref = ref_frame->Twc_;
  Eigen::Matrix3d R, R_cur, R_ref = T_ref.rotationMatrix();
  Eigen::Vector3d t, t_cur, t_ref = T_ref.translation();
  cv::cv2eigen(dR, R);
  cv::cv2eigen(dt, t);

  // 将平移尺度归一化
  t.normalize();

  // 更新当前帧
  R_cur = R_ref * R;
  t_cur = t_ref + scale * (R_ref * t);
  cur_frame->Twc_ = Sophus::SE3d(R_cur, t_cur);
}

}  // namespace app
}  // namespace pose_correction
