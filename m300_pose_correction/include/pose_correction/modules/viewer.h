/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: viewer.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_VIEWER_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_VIEWER_H_

#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>
#include <memory>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

namespace pose_correction {
namespace modules {
class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Viewer> Ptr;

  typedef std::vector<Eigen::Isometry3d,
                      Eigen::aligned_allocator<Eigen::Isometry3d>>
      TrajType;

  Viewer();

  void Stop();

  void Update(const TrajType traj);

 private:
  void DrawTrajectory();

  std::mutex traj_mutex_;
  TrajType traj_;

  std::thread viewer_thread_;
  std::atomic<bool> viewer_running_;
};

}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_VIEWER_H_
