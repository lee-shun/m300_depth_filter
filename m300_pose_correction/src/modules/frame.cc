/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-29
 *
 *   @Description:
 *
 *******************************************************************************/

#include "pose_correction/modules/frame.h"
#include "pose_correction/modules/ORBextractor.h"

namespace pose_correction {
namespace modules {

bool Frame::DetectFeatures() {
  if (img_.empty()) {
    std::cerr << "empty image!" << std::endl;
    return false;
  }

  // 清空上一次的检测
  keypoints_.clear();

  auto orb_extractor_ptr =
      std::make_shared<ORB::ORBextractor>(1000, 1.2, 8, 20, 7);
  orb_extractor_ptr->operator()(img_, cv::Mat(), keypoints_, descriptors_);

  return true;
}

}  // namespace modules
}  // namespace pose_correction
