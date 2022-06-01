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

void PoseCorrector::TrackFeatures(const modules::Frame::Ptr ref_frame,
                                  const modules::Frame::Ptr cur_frame,
                                  std::vector<cv::Point2f>& tracked_pts_ref,
                                  std::vector<cv::Point2f>& tracked_pts_cur) {
  // 转换为pts
  std::vector<cv::Point2f> pts_ref = ref_frame->kps_pt_,
                           pts_cur = cur_frame->kps_pt_;

  std::vector<float> error;
  std::vector<uchar> status;
  cv::Size win_size = cv::Size(21, 21);
  cv::TermCriteria term_crit = cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

  cv::calcOpticalFlowPyrLK(ref_frame->img_, cur_frame->img_, pts_ref, pts_cur,
                           status, error, win_size, 3, term_crit);

  // 根据status填充跟踪到的特征点
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] == 1) {
      tracked_pts_ref.push_back(pts_ref[i]);
      tracked_pts_cur.push_back(pts_cur[i]);
    }
  }

  // 绘制光流点
  cv::Mat img_to_draw;
  cv::cvtColor(cur_frame->img_.clone(), img_to_draw, cv::COLOR_GRAY2BGR);
  for (int i = 0; i < tracked_pts_cur.size(); ++i) {
    cv::circle(img_to_draw, tracked_pts_cur[i], 2, cv::Scalar(0, 250, 0), 2);
    cv::line(img_to_draw, tracked_pts_cur[i], tracked_pts_ref[i],
             cv::Scalar(0, 250, 0), 2);
  }
  cv::imshow("current image", img_to_draw);
  cv::waitKey(1);
}

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
