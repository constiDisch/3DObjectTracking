// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/loader_camera.h>
#include <m3t/manual_detector.h>
#include <m3t/static_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/region_modality.h>
#include <m3t/renderer_geometry.h>
#include <m3t/tracker.h>
#include <Eigen/Geometry>
#include <memory>

// Example script for the detector usage on the data provided in data/sequence
// with the object triangle
int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Not enough arguments: Provide camera metafile, body "
                 "metafile, detector metafile, temp directory";
    return -1;
  }
  const std::filesystem::path base_directory{argv[1]};
  const std::filesystem::path color_camera_metafile_path = base_directory / "color_camera.yaml";
  const std::filesystem::path body_metafile_path = base_directory / "body.yaml";
  const std::filesystem::path detector_metafile_path = base_directory / "detector.yaml";
  const std::filesystem::path region_model_metafile_path = base_directory / "region_model.yaml";
  const std::filesystem::path region_modality_metafile_path = base_directory / "region_modality.yaml";
  
  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  // Set up camera
  auto camera_ptr{std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};

  // Set up viewers
  auto viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "viewer", camera_ptr, renderer_geometry_ptr)};
  viewer_ptr->StartSavingImages(base_directory);
  tracker_ptr->AddViewer(viewer_ptr);

  // Set up body triangle
  auto body_ptr{std::make_shared<m3t::Body>("triangle", body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Set up region mode
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      "region_model", region_model_metafile_path, body_ptr)};

  // Set up region modality
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", region_modality_metafile_path, body_ptr, camera_ptr, region_model_ptr)};

  // Set up link
  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);

  // Set up optimizer
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  // Set up detector
  //auto detector_ptr{std::make_shared<m3t::ManualDetector>(
   //    "detector", detector_metafile_path, optimizer_ptr, camera_ptr)};
  auto detector_ptr{std::make_shared<m3t::StaticDetector>("detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);
  //tracker_ptr->set_viewer_time(3000);

  // Start tracking
  if (!tracker_ptr->SetUp()) return -1;
  
  // simply run tracking
  tracker_ptr->RunTrackerProcess(true, true);
  
  auto result = body_ptr->body2world_pose();

  std::filesystem::path path{base_directory / "pose_out.txt"};
  std::ofstream ofs{path, std::ios::out | std::ios::binary};
  m3t::WriteValueToTxt(ofs, "PoseOut", result);
  ofs.flush();
  ofs.close();
  std::cout << "Output pose saved " << path << std::endl;

  return 0;
}
