// SPDX-License-Identifier: MIT
// Copyright: Constantin Disch


#ifndef M3T_INCLUDE_M3T_OPENCV_VIDEOCAP_CAMERA_H_
#define M3T_INCLUDE_M3T_OPENCV_VIDEOCAP_CAMERA_H_

#include <m3t/camera.h>
#include <m3t/common.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace m3t {

/**
 * \brief \ref Camera that allows getting color images from an device support by the opencv video io
 *
 * @param image_scale scales images to avoid borders after rectification.
 * @param use_depth_as_world_frame specifies the depth camera frame as world
 * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
 */
class VideoCapCamera : public ColorCamera {
 public:

  VideoCapCamera(const std::string &name, const std::filesystem::path &metafile_path);
  VideoCapCamera(const std::string &name, const Intrinsics &intrinsics, const int device_id=0, const int api_id = 0);

  bool SetUp() override;

  // Main method
  bool UpdateImage(bool synchronized) override;

  private:
    int device_id_;
    int api_id_;

    cv::VideoCapture cap;
    bool LoadMetaData();
    void SaveMetaDataIfDesired() const;

};



}

#endif  // M3T_INCLUDE_M3T_AZURE_KINECT_CAMERA_H_
