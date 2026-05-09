#include "sh_perception_server/perception_server.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"

namespace sh_perception_server
{

PerceptionServer::PerceptionServer(const std::string & node_name) :
  rclcpp_lifecycle::LifecycleNode(node_name),
  detector_loader_("sh_base_template", "sh_base_template::DetectorBase"),
  localiser_loader_("sh_base_template", "sh_base_template::LocaliserBase"),
  grasp_loader_("sh_base_template", "sh_base_template::GraspGeneratorBase")
{
  RCLCPP_INFO(get_logger() , "Initializing");

  if (!has_parameter("detector.plugin")) {
    declare_parameter<std::string>("detector.plugin", "sh_perception_server::YoloDetector");
  }
  get_parameter<std::string>("detector.plugin", detector_plugin_name_);
  detector_ = detector_loader_.createUniqueInstance(detector_plugin_name_);
  RCLCPP_INFO(get_logger() , "\033[1;32m[%s] loaded.\033[0m", detector_plugin_name_.c_str());

  if (!has_parameter("localiser.plugin")) {
    declare_parameter<std::string>("localiser.plugin", "sh_default_plugins::MeanLocaliser");
  }
  get_parameter<std::string>("localiser.plugin", localiser_plugin_name_);
  localiser_ = localiser_loader_.createUniqueInstance(localiser_plugin_name_);
  RCLCPP_INFO(get_logger() , "\033[1;32m[%s] loaded.\033[0m", localiser_plugin_name_.c_str());

  if (!has_parameter("grasp.plugin")) {
    declare_parameter<std::string>("grasp.plugin", "sh_default_plugins::DefaultGraspGenerator");
  }
  get_parameter<std::string>("grasp.plugin", grasping_plugin_name_);
  grasp_ = grasp_loader_.createUniqueInstance(grasping_plugin_name_);
  RCLCPP_INFO(get_logger() , "\033[1;32m[%s] loaded.\033[0m", grasping_plugin_name_.c_str());
}

CallbackReturn PerceptionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger() , "Configuring.");

  declare_parameters();

  if (!detector_->configure(weak_from_this())) {
    return CallbackReturn::FAILURE;
  }

  if (!localiser_->configure(weak_from_this())) {
    return CallbackReturn::FAILURE;
  }

  if (!grasp_->configure(weak_from_this())) {
    return CallbackReturn::FAILURE;
  }

  double a;
  a = get_parameter_or<double>("asd", 2.0);
  declare_parameter<double>("aassgsd", 3.0);

  RCLCPP_INFO(get_logger() , "Configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger() , "Activating.");

  service_ = this->create_service<sh_interfaces::srv::RunPerception>(
    "get_detections",
    std::bind(
      &PerceptionServer::trigger_detections_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  if (publish_detections_image_) {
    image_publisher_ = this->create_publisher<ImageMsg>("detections", 10);
  }

  RCLCPP_INFO(get_logger() , "Activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  service_.reset();
  if (publish_detections_image_) {
    image_publisher_.reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void) state;
  detector_->cleanup();
  localiser_->cleanup();
  grasp_->cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void) state;
  return CallbackReturn::SUCCESS;
}

void PerceptionServer::trigger_detections_callback(
  const std::shared_ptr<sh_interfaces::srv::RunPerception::Request> request,
  std::shared_ptr<sh_interfaces::srv::RunPerception::Response> response)
{
  (void) request;
  (void) response;

  RCLCPP_INFO(get_logger() , "Request received.");

  cv::Mat input = cv_bridge::toCvShare(
    std::make_shared<sensor_msgs::msg::Image>(request->rgb_image), "bgr8")->image;
  std::vector<sh_interfaces::msg::Detection2D> detections;

  // Detection Stage
  if (!detector_->detect(input, detector_base_config_, detections)) {
    response->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Detected objects: %ld", detections.size());

  // Localisation Stage
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(detections.size());

  cv::Mat depth_input = cv_bridge::toCvShare(
    std::make_shared<ImageMsg>(request->depth_image), request->depth_image.encoding)->image;
  auto k = request->cam_info.k;
  std::array<double, 4UL> camera_intrinsics = {
    k[0], k[4], k[2], k[5]};

  if (!localiser_->localise(depth_input, request->depth_image.encoding, camera_intrinsics, detections, poses)) {
    response->success = false;
    return;
  }

  // Grasping Generator
  if (!grasp_->generate_grasp(poses)) {
    response->success = false;
    return;
  }

  int n_objects = detections.size();
  response->success = true;
  response->detected_objects.header = request->rgb_image.header;
  response->detected_objects.num_objects = n_objects;
  response->detected_objects.objects.resize(n_objects);

  for (int i = 0; i < n_objects; ++i) {
    response->detected_objects.objects[i].class_id = detections[i].class_id;
    response->detected_objects.objects[i].confidence = detections[i].confidence;
    response->detected_objects.objects[i].pose = poses[i];
  }

  if (publish_detections_image_) {
    annotate_ouput(input, detections);
    publish_output(input);
  }
}

void PerceptionServer::declare_parameters()
{
  if (!has_parameter("model_path")) {
    auto perception_pkg = ament_index_cpp::get_package_share_directory("sh_perception_server");
    declare_parameter<std::string>("model_path", perception_pkg+"/model/apple_mobi.pt");
  }
  get_parameter("model_path", detector_base_config_.model_path);

  if (!has_parameter("input_size")) {
    declare_parameter<std::vector<int64_t>>(
      "input_size",
      std::vector<int64_t>{1280, 736});
  }
  get_parameter("input_size", detector_base_config_.input_size);

  if (!has_parameter("use_gpu")) {
    declare_parameter<bool>("use_gpu", true);
  }
  get_parameter("use_gpu", detector_base_config_.use_gpu);

  if (!has_parameter("inference_verbose")) {
    declare_parameter<bool>("inference_verbose", false);
  }
  get_parameter("inference_verbose", detector_base_config_.inference_verbose);

  if (!has_parameter("classes")) {
    declare_parameter<std::vector<std::string>>("classes", std::vector<std::string>{"Apple"});
  }
  get_parameter("classes", classes_);

  if (!has_parameter("publish_detections_image")) {
    declare_parameter<bool>("publish_detections_image", false);
  }
  get_parameter("publish_detections_image", publish_detections_image_);
}

void PerceptionServer::annotate_ouput(
  cv::Mat& output,
  const std::vector<sh_interfaces::msg::Detection2D>& detections)
{
  for (const auto& detection: detections) {
    cv::Rect rect(
      static_cast<int>(detection.x1),
      static_cast<int>(detection.y1),
      static_cast<int>(detection.x2-detection.x1),
      static_cast<int>(detection.y2-detection.y1));

    cv::rectangle(output, rect, cv::Scalar(0, 255, 0), 2);

    cv::putText(
      output,
      classes_[detection.class_id],
      cv::Point(static_cast<int>(detection.x1), static_cast<int>(detection.y1) - 5),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(255, 0, 0),
      2);

  }
}

void PerceptionServer::publish_output(const cv::Mat& output)
{
  auto output_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    output).toImageMsg();

  image_publisher_->publish(*output_msg);


}

}  // namespace sh_perception_server