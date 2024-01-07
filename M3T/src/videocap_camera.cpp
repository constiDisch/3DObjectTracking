// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/videocap_camera.h>

#include <opencv2/core/eigen.hpp>

namespace m3t {

    VideoCapCamera::VideoCapCamera(
        const std::string &name, 
        const std::filesystem::path &metafile_path
    ): ColorCamera{name, metafile_path}{};

    VideoCapCamera::VideoCapCamera(
        const std::string &name, 
        const Intrinsics &intrinsics,
        const int device_id, 
        const int api_id
        ): ColorCamera{name}, device_id_{device_id}, api_id_{api_id}{
            intrinsics_ = intrinsics;
        };


    bool VideoCapCamera::SetUp() {
        set_up_ = false;
        if (!metafile_path_.empty())
            if (!LoadMetaData()) return false;
        SaveMetaDataIfDesired();

        cap.open(device_id_, api_id_);
        if (!cap.isOpened()){
            std::cerr << "Could not open video capture" << std::endl;
            return false;
        }

        if (
            !cap.set(cv::CAP_PROP_FRAME_WIDTH, intrinsics_.width) || 
            !cap.set(cv::CAP_PROP_FRAME_HEIGHT, intrinsics_.height)
        ){
            std::cerr << "Could not set desired width and height" << std::endl;
            return false;
        }
        set_up_ = true;
        return UpdateImage(true);
    };

    bool VideoCapCamera::UpdateImage(bool synchronized){

        // ToDo: Check what synchronized really means.
        if (!set_up_){
            std::cerr << "Setup camera first" << std::endl;
            return false;
        }

        // load image here

        cv::Mat undistorted_image;
        cap.read(undistorted_image);
        if (undistorted_image.empty()){
            std::cerr << "Could not retrieve image" << std::endl;
        }
        image_ = undistorted_image;
        return true;
    };

    void VideoCapCamera::SaveMetaDataIfDesired() const {
        if (save_images_) {
            std::filesystem::path path{save_directory_ / (name_ + ".yaml")};
            cv::FileStorage fs{path.string(), cv::FileStorage::WRITE};
            fs << "device_id" << device_id_;
            fs << "api_id" << api_id_;
            WriteValueToYaml(fs, "intrinsics", intrinsics_);
            WriteValueToYaml(fs, "camera2world_pose", camera2world_pose_);
            fs.release();
        }
    };

    bool VideoCapCamera::LoadMetaData() {
        // Open file storage from yaml
        cv::FileStorage fs;
        if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

        // Read parameters from yaml
        if (!(
            ReadRequiredValueFromYaml(fs, "device_id", &device_id_) &&
            ReadRequiredValueFromYaml(fs, "api_id", &api_id_) &&
            ReadRequiredValueFromYaml(fs, "intrinsics", &intrinsics_))) {
            std::cerr << "Could not read all required body parameters from "
                    << metafile_path_ << std::endl;
            return false;
        }
        ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
        ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
        ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
        ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
        ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
        fs.release();

        // Process parameters
        if (save_directory_.is_relative())
            save_directory_ = metafile_path_.parent_path() / save_directory_;
        world2camera_pose_ = camera2world_pose_.inverse();
        return true;
    }



}  // namespace m3t
