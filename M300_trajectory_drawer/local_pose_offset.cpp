/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: local_pose_offset.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-18
 *
 *   @Description: take 17.1 gps as start point, convert the other distance into
 *   local pose.
 *
 *******************************************************************************/
#include <Eigen/Core>
#include "./CommonTypes.hpp"
#include "./MathLib.hpp"
#include "FileWritter.hpp"

bool ReadPosition(const std::string filename, const int line_index,
                  Eigen::Vector3d* position) {
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
  for (int i = 0; i < 4; ++i) {
    if (!getline(fin, pose_tmp, ',')) {
      PRINT_ERROR("pose reading error! at line_index %d", line_index);
      return false;
    }
    // PRINT_DEBUG("read trans:index+xyz:%.8f", std::stod(trans_tmp));
    pose_elements.push_back(std::stod(pose_tmp));
  }

  Eigen::Vector3d t;
  t << pose_elements[1], pose_elements[2], pose_elements[3];

  (*position) = t;

  return true;
}

int main(int argc, char** argv) {
  PRINT_INFO("usage: ref_folder cur_folder");
  std::string ref_folder =
      "/home/ls/m300_depth_filter/M300_trajectory_drawer/17.1";
  std::string cur_folder =
      "/home/ls/m300_depth_filter/M300_trajectory_drawer/51.2";

  std::string ref_gps_file = ref_folder + "/gps.csv";
  std::string cur_gps_file = cur_folder + "/gps.csv";

  // read the ref and cur home pose
  // lon, lat, alt
  Eigen::Vector3d ref_gps, cur_gps;
  ReadPosition(ref_gps_file, 1, &ref_gps);
  ReadPosition(cur_gps_file, 1, &cur_gps);

  // calculate offset
  double ref_gps_d[2] = {ref_gps[0], ref_gps[1]},
         cur_gps_d[2] = {cur_gps[0], cur_gps[1]};
  double offset[2];
  FFDS::TOOLS::LatLong2Meter<double>(ref_gps_d, cur_gps_d, offset);

  Eigen::Vector3d off_ned;
  off_ned << offset[0], offset[1], cur_gps[2] - ref_gps[2];

  // apply offset to the whole local file
  std::string cur_local_pose_file = cur_folder + "/local_pose.csv";
  int line_index = 1;
  Eigen::Vector3d cur_local_enu, cur_gps_enu;
  depth_filter::FileWritter new_local_writter(
      cur_folder + "/new_local_pose.csv", 8);
  new_local_writter.new_open();
  new_local_writter.write("index", "x", "y", "z");

  while (ReadPosition(cur_local_pose_file, line_index, &cur_local_enu)) {
    ReadPosition(cur_gps_file, line_index, &cur_gps_enu);
    line_index++;
    // frame index, E, N, U
    new_local_writter.write(line_index - 1, cur_local_enu[0] + off_ned[1],
                            cur_local_enu[1] + off_ned[0],
                            -cur_gps_enu[2]);
  }
  return 0;
}
