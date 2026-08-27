#include "sh_rgbd_perception_pipeline/rgbd_perception_pipeline.hpp"

#include <chrono>

#include "cv_bridge/cv_bridge.hpp"

#include "sh_base_template/types/perception_types.hpp"
#include "sh_utils/ros2_node_utils.hpp"

namespace sh_rgbd_perception_pipeline
{

RgbdPerceptionPipeline::RgbdPerceptionPipeline() :
  detector_loader_("sh_base_template", "sh_base_template::DetectorBase"),
  localiser_loader_("sh_base_template", "sh_base_template::LocaliserBase"),
  grasp_loader_("sh_base_template", "sh_base_template::GraspGeneratorBase")
{
}

RgbdPerceptionPipeline::~RgbdPerceptionPipeline()
{
  reset_plugins();
  perception_publisher_.reset();
}

bool RgbdPerceptionPipeline::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_node, "pipeline.detector.plugin",
    detector_plugin_name_, "sh_rgbd_perception_pipeline::YoloDetectorPY");

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_node, "pipeline.localiser.plugin",
    localiser_plugin_name_, "sh_rgbd_perception_pipeline::MeanLocaliser");

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_node, "pipeline.grasp.plugin",
    grasping_plugin_name_, "sh_rgbd_perception_pipeline::DefaultGraspGenerator");

  try {
    detector_ = detector_loader_.createUniqueInstance(detector_plugin_name_);
    RCLCPP_INFO(
      shared_node->get_logger() , "\033[1;32m[%s] loaded.\033[0m", detector_plugin_name_.c_str());

    localiser_ = localiser_loader_.createUniqueInstance(localiser_plugin_name_);
    RCLCPP_INFO(
      shared_node->get_logger() , "\033[1;32m[%s] loaded.\033[0m", localiser_plugin_name_.c_str());

    grasp_ = grasp_loader_.createUniqueInstance(grasping_plugin_name_);
    RCLCPP_INFO(
      shared_node->get_logger() , "\033[1;32m[%s] loaded.\033[0m", grasping_plugin_name_.c_str());
  } catch (const pluginlib::PluginlibException& e) {
    RCLCPP_ERROR(shared_node->get_logger(), "Error loading [%s]: ", e.what());
    reset_plugins();
    return false;
  }

  if (!detector_->configure(node)) {
    RCLCPP_ERROR(shared_node->get_logger(), "Detector configuration failed.");
    reset_plugins();
    return false;
  }

  if (!localiser_->configure(node)) {
    RCLCPP_ERROR(shared_node->get_logger(), "Localiser configuration failed.");
    reset_plugins();
    return false;
  }

  if (!grasp_->configure(node)) {
    RCLCPP_ERROR(shared_node->get_logger(), "Grasp generator configuration failed.");
    reset_plugins();
    return false;
  }

  perception_publisher_ = shared_node->create_publisher<sensor_msgs::msg::Image>(
    "perception_image", 10);

  return true;
}


void RgbdPerceptionPipeline::activate()
{
  detector_->activate();
  localiser_->activate();
  grasp_->activate();
}

void RgbdPerceptionPipeline::deactivate()
{
  detector_->deactivate();
  localiser_->deactivate();
  grasp_->deactivate();
}

void RgbdPerceptionPipeline::cleanup()
{
  detector_->cleanup();
  localiser_->cleanup();
  grasp_->cleanup();
  reset_plugins();
  perception_publisher_.reset();
}

bool RgbdPerceptionPipeline::process(
  const sh_interfaces::msg::PerceptionInput& input,
  sh_interfaces::msg::PerceptionScene& output)
{
  auto start = std::chrono::steady_clock::now();

  // Check empty or unmatched input RGB-D
  if ((input.rgb_images.size() != input.depth_images.size()) ||
    (input.rgb_images.size() != input.camera_infos.size()) ||
    (!input.rgb_images.size()))
  {
    return false;
  }
  int cam_size = input.rgb_images.size();

  // ************************* DETECTOR *************************
  sh_base_template::types::DetectorInput detector_input;
  sh_base_template::types::DetectorOutput detector_output;

  detector_input.frames.resize(cam_size);

  cv_bridge::CvImageConstPtr cv_rgb_ptr;
  cv_bridge::CvImageConstPtr cv_depth_ptr;
  for (int i = 0; i < cam_size; ++i) {
    cv_rgb_ptr = cv_bridge::toCvShare(std::make_shared<ImageMsg>(input.rgb_images[i]), "bgr8");
    cv_depth_ptr = cv_bridge::toCvShare(
      std::make_shared<ImageMsg>(input.depth_images[i]), input.depth_images[i].encoding);
    detector_input.frames[i].rgb = cv_rgb_ptr;
    detector_input.frames[i].depth = cv_depth_ptr;
  }

  if (!detector_->detect(detector_input, detector_output)) {
    return false;
  }

  for (const auto& output: detector_output.output) {
    std::cout << output.class_name << " [" << output.bbox.x << ", " << output.bbox.y << "]" << std::endl;
  }

  // ************************* LOCALIZER *************************
  sh_base_template::types::LocaliserInput localiser_input;
  sh_base_template::types::LocaliserOutput localiser_output;

  localiser_input.frame.camera_info = input.camera_infos[0];
  localiser_input.frame.depth = cv_depth_ptr;
  localiser_input.detections = detector_output;

  if (!localiser_->localise(localiser_input, localiser_output)) {
    return false;
  }

  for (const auto& pose: localiser_output.poses) {
    std::cout << "Pose XYZ [" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << "]" << std::endl;
  }

  // ************************* GRASP *************************
  if (!grasp_->generate_grasp(localiser_input.frame, localiser_output.poses)) {
    return false;
  }

  for (const auto& pose: localiser_output.poses) {
    std::cout << "Pose WXYZ [" << pose.pose.orientation.w << ", " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", " << pose.pose.orientation.z << "]" << std::endl;
  }

  int n_objects = detector_output.output.size();
  output.header = input.rgb_images[0].header;
  output.objects.resize(n_objects);

  for (int i = 0; i < n_objects; ++i) {
    output.objects[i].class_id = detector_output.output[i].class_id;
    output.objects[i].class_name = detector_output.output[i].class_name;
    output.objects[i].confidence = detector_output.output[i].confidence;
    output.objects[i].pose = localiser_output.poses[i].pose;
  }

  show_image(cv_rgb_ptr->image, detector_output);

  auto end = std::chrono::steady_clock::now();
  output.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  return true;
}

void RgbdPerceptionPipeline::show_image(
  cv::Mat rgb_image,
  const sh_base_template::types::DetectorOutput& detector_output)
{
  cv::Mat rgb_for_publish;
  cv::cvtColor(rgb_image, rgb_for_publish, cv::COLOR_BGR2RGB);
  for (const auto& output: detector_output.output) {
    cv::Rect rect(
      output.bbox.x - output.bbox.width/2,
      output.bbox.y - output.bbox.height/2,
      output.bbox.width,
      output.bbox.height);
    cv::rectangle(rgb_for_publish, rect, cv::Scalar(0, 255, 0), 2);
  }

  sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "rgb8",
    rgb_for_publish).toImageMsg();

  perception_publisher_->publish(*img_msg);
}

}  // namespace sh_rgbd_perception_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_rgbd_perception_pipeline::RgbdPerceptionPipeline, sh_base_template::PerceptionPipelineBase)