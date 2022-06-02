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
#include <memory>
#include <opencv2/core/eigen.hpp>

namespace pose_correction {
namespace app {

PoseCorrector::PoseCorrector() {
  dataset_ = std::make_shared<modules::Dataset>(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");
  camera_ = std::make_shared<modules::Camera>(
      "/home/ls/m300_depth_filter/H20T_visiable_calbi/H20T_visible_param.yaml");
  viewer_ = std::make_shared<modules::Viewer>();

  feat_matcher_ = std::make_shared<modules::FeatureMatcher>();
}

void PoseCorrector::Run() {
  std::vector<cv::String> imgs_paths;
  dataset_->GetAllImageNames(imgs_paths);
  std::vector<double> frame_scales;
  dataset_->GetGroundTruthScale(frame_scales);

  if (imgs_paths.empty() || frame_scales.empty() ||
      (imgs_paths.size() != frame_scales.size())) {
    std::cerr << "读取数据集有误!" << std::endl;
    std::cerr << "图像数据非空:" << imgs_paths.empty() << std::endl;
    std::cerr << "轨迹数据非空:" << frame_scales.empty() << std::endl;
    std::cerr << "图像与轨迹数相等:"
              << (imgs_paths.size() != frame_scales.size()) << std::endl;
    return;
  }

  // first frame
  auto ref_frame = modules::Frame::CreateFrame();
  ref_frame->img_ = cv::imread(imgs_paths[0], cv::IMREAD_GRAYSCALE);
  ref_frame->DetectFeatures();

  // trajectory
  modules::Viewer::TrajType traj;

  for (size_t i = 1; i < imgs_paths.size(); ++i) {
    // 初始化当前帧
    auto cur_frame = modules::Frame::CreateFrame();
    cur_frame->img_ = cv::imread(imgs_paths[i], cv::IMREAD_GRAYSCALE);
    cur_frame->DetectFeatures();

    // feature maching
    std::vector<cv::Point2f> tracked_pts_ref;
    std::vector<cv::Point2f> tracked_pts_cur;
    feat_matcher_->MacthFeaturesBF(ref_frame, cur_frame, tracked_pts_ref,
                                   tracked_pts_cur);

    // 对极几何更新当前帧的位姿
    EstimatePose(tracked_pts_ref, tracked_pts_cur, frame_scales[i], ref_frame,
                 cur_frame);

    // 将当前帧位姿存入, 更新显示
    if (viewer_) {
      Eigen::Matrix3d R_cur = cur_frame->Twc_.rotationMatrix();
      Eigen::Vector3d t_cur = cur_frame->Twc_.translation();
      Eigen::Isometry3d T(R_cur);
      T.pretranslate(t_cur);
      traj.push_back(T);
      viewer_->Update(traj);
    }

    // 将当前帧作为为下一帧的参考帧
    ref_frame = cur_frame;
  }
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
