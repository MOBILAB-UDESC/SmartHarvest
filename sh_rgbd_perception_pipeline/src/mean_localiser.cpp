#include "sh_rgbd_perception_pipeline/mean_localiser.hpp"

#include <cmath>

#include "Eigen/Dense"
#include "sh_base_template/types/perception_types.hpp"
#include "sh_utils/ros2_node_utils.hpp"

namespace sh_rgbd_perception_pipeline
{

bool MeanLocaliser::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  sh_utils::ros2_node_utils::declare_and_get_parameter<int>(
    shared_node, "pipeline.localiser.points",
    points_, 1);

  if (points_ < 1) {
    RCLCPP_WARN(
      shared_node->get_logger(),
      "Minimum points is 1.");
    points_ = 1;
  }

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<double>>(
    shared_node, "pipeline.localiser.camera_to_end_effector_transform",
    camera_to_end_effector_transform_, std::vector<double>{0.0, 0.0, 0.0});

  if (camera_to_end_effector_transform_.size() != 3) {
    RCLCPP_WARN(
      shared_node->get_logger(),
      "'camera_to_end_effector_transform_' param in localiser must be a 3D vector. Using default zero vector.");
    camera_to_end_effector_transform_.clear();
    camera_to_end_effector_transform_ = std::vector<double>{0.0, 0.0, 0.0};
  }

  return true;
}

void MeanLocaliser::cleanup()
{
}

bool MeanLocaliser::localise(
  const sh_base_template::types::LocaliserInput& localiser_input,
  sh_base_template::types::LocaliserOutput& localiser_output)
{
  depth_encoding_ = localiser_input.frame.depth->encoding;

  auto k = localiser_input.frame.camera_info.k;
  camera_intrinsics_.f_x = k[0];
  camera_intrinsics_.f_y = k[4];
  camera_intrinsics_.c_x = k[2];
  camera_intrinsics_.c_y = k[5];

  sh_base_template::types::PoseFeatures pose;
  localiser_output.poses.reserve(localiser_input.detections.output.size());

  for (const auto& detection: localiser_input.detections.output) {
    auto xyz = compute_xyz(
      localiser_input.frame.depth->image,
      {detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height});

    if (!xyz) {
      pose.valid_pose = false;
      localiser_output.poses.push_back(pose);
      continue;
    }

    pose.valid_pose = true;
    pose.pose.position.x = (*xyz)[0];
    pose.pose.position.y = (*xyz)[1];
    pose.pose.position.z = (*xyz)[2];

    localiser_output.poses.emplace_back(pose);
  }

  return true;
}

std::optional<std::array<double, 3>> MeanLocaliser::compute_xyz(
  const cv::Mat& depth_input,
  const std::array<double, 4>& bbox)
{
  int x_center = bbox[0];
  int y_center = bbox[1];

  std::vector<std::array<int, 2>> xy;
  int n = 4; // 8 for diagonals also.
  xy.resize(n*points_ + 1);
  xy[n*points_] = {x_center, y_center};

  for (int i=0; i<points_; ++i) {
    xy[n*i] = {x_center-i-1, y_center};
    xy[n*i+1] = {x_center-i+1, y_center};
    xy[n*i+2] = {x_center, y_center-i-1};
    xy[n*i+3] = {x_center, y_center-i+1};
    // xy[n*i+4] = {x_center-i-1, y_center-i-1};
    // xy[n*i+5] = {x_center-i-1, y_center-i+1};
    // xy[n*i+6] = {x_center-i+1, y_center-i-1};
    // xy[n*i+7] = {x_center-i+1, y_center-i+1};
  }

  double z0 = 0.0;
  int valid_samples = 0;
  double depth = 0.0;
  for (const auto& point: xy) {
    if (depth_encoding_ == "32FC1") {
      depth_scaling_ = 1.0;
      depth = depth_input.at<float>(point[1], point[0]);
    } else {
      depth = depth_input.at<uint16_t>(point[1], point[0]);
      depth_scaling_ = 0.001;
    }
    if (depth > 0.0) {
      z0 += depth*depth_scaling_;
      ++valid_samples;
    }
  }

  if (!valid_samples) {
    return std::nullopt;
  }

  z0 = z0/valid_samples;
  int w = bbox[2];
  int h = bbox[3];

  // Estimate sphere radius if treating object as spherical (e.g., apples)
  double r = (w>h) ? (z0 * w / (2 * camera_intrinsics_.f_x)) : (z0 * h / (2 * camera_intrinsics_.f_y));

  double X = z0 * (x_center - camera_intrinsics_.c_x) / camera_intrinsics_.f_x;
  double Y = z0 * (y_center - camera_intrinsics_.c_y) / camera_intrinsics_.f_y;
  double Z = z0 + r;

  // TODO: use TF ros pḱg to dynamically get camera_to_end_effector_transform_ vector.

  Eigen::AngleAxisd rx(camera_to_end_effector_transform_[0],  Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(camera_to_end_effector_transform_[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(camera_to_end_effector_transform_[2],   Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R_cam_to_eef =
      (rz * ry * rx).toRotationMatrix();
  Eigen::Vector3d pEef(X, Y, Z);
  Eigen::Vector3d pCam = R_cam_to_eef * pEef;

  return std::array<double, 3>{pCam.x(), pCam.y(), pCam.z()};
}

}  // namespace sh_rgbd_perception_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_rgbd_perception_pipeline::MeanLocaliser, sh_base_template::LocaliserBase)