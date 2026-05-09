#include "sh_default_plugins/perception_plugin/mean_localiser.hpp"

namespace sh_default_plugins
{

bool MeanLocaliser::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  if (!shared_node->has_parameter("localiser.points")) {
    shared_node->declare_parameter<int>("localiser.points", 1);
  }
  shared_node->get_parameter("localiser.points", points_);

  shared_node->get_parameter("use_sim_time", use_sim_time_);

  return true;
}

void MeanLocaliser::cleanup()
{
}

bool MeanLocaliser::localise(
  const cv::Mat& depth_input,
  const std::string& depth_scale,
  const std::array<double, 4UL>& cam_intrinsics,
  const std::vector<sh_interfaces::msg::Detection2D>& detections,
  std::vector<geometry_msgs::msg::Pose>& poses)
{
  geometry_msgs::msg::Pose pose;

  // int valid_detections = 0;
  for (const auto& detection: detections) {
    auto xyz = compute_xyz(
      depth_input,
      {detection.x1, detection.x2, detection.y1, detection.y2},
      cam_intrinsics);

    if (!xyz) {
      continue;
    }

    pose.position.x = (*xyz)[0];
    pose.position.y = (*xyz)[1];
    pose.position.z = (*xyz)[2];

    std::cout << "X: " << (*xyz)[0] << " Y: " << (*xyz)[1] << " Z: " << (*xyz)[2] << std::endl;

    // ++valid_detections;

    poses.push_back(pose);
  }

  // poses.resize(valid_detections);

  return true;
}

std::optional<std::array<double, 3>> MeanLocaliser::compute_xyz(
  const cv::Mat& depth_input,
  const std::array<double, 4>& bbox,
  const std::array<double, 4UL>& cam_intrinsics)
{
  double fx = cam_intrinsics[0];
  double fy = cam_intrinsics[1];
  double cx = cam_intrinsics[2];
  double cy = cam_intrinsics[3];

  int x_center = int((bbox[0] + bbox[1]) / 2);
  int y_center = int((bbox[2] + bbox[3]) / 2);

  std::vector<std::array<int, 2>> xy;
  int n = 4;
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
    // return std::nullopt;
    return std::array<double, 3>{0.0, 0.0, 0.0};
  }

  z0 = z0/valid_samples;
  double w = bbox[1] - bbox[0];
  double h = bbox[3] - bbox[2];

  // Estimate sphere radius if treating object as spherical (e.g., apples)
  double r = (w>h) ? (z0 * w / (2 * fx)) : (z0 * h / (2 * fy));

  double X = z0 * (x_center - cx) / fx;
  double Y = z0 * (y_center - cy) / fy;
  double Z = z0 + r;

  if (use_sim_time_) {
    return std::array<double, 3>{Z, -X, -Y};
  }

  return std::array<double, 3>{X, Y, Z};
}

}  // namespace sh_default_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_default_plugins::MeanLocaliser, sh_base_template::LocaliserBase)