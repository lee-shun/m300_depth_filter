/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: AbsolutePoseRecover.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-05
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/AbsolutePoseRecover.hpp"
#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/FileWritter.hpp"

namespace depth_filter {
bool AbsolutePoseRecover::Recover() {
  FileWritter pose_writer("abs_rel_pose.csv", 9);
  pose_writer.new_open();

  for (int line_index = 0; line_index < 100; ++line_index) {
    Sophus::SE3d Tcw, Trw;
    Eigen::Vector3d abs_trans, abs_trans_ref;

    int frame_index = 0;
    if (!ReadPose(pose_filename_, line_index, &frame_index, &Tcw)) break;

    // found the line_index of the beginning frame
    if (frame_index < ref_frame_index_) {
      continue;
    } else if (frame_index == ref_frame_index_) {
      Trw = Tcw;
      ReadTranslation(abs_trans_filename_, ref_frame_index_, &abs_trans_ref);
    }

    if (!ReadTranslation(abs_trans_filename_, frame_index, &abs_trans)) break;

    // recover realtive Trc
    double scale = (abs_trans - abs_trans_ref).norm();
    Sophus::SE3d Trc = Trw * Tcw.inverse();
    Trc.translation() = scale * Trc.translation().normalized();

    // wirte
    Eigen::Vector3d t = Trc.translation();
    Eigen::Matrix3d R = Trc.rotationMatrix();
    pose_writer.write(frame_index, R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1),
                      R(1, 2), R(2, 0), R(2, 1), R(2, 2), t(0), t(1), t(2));
  }
  return true;
}

}  // namespace depth_filter
