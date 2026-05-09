#include "sh_default_plugins/perception_plugin/yolo_detector_py.hpp"

#include "cv_bridge/cv_bridge.hpp"

namespace sh_default_plugins
{

bool YoloDetectorPY::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  node_ = node;
  auto shared_node = node_.lock();

  if (!shared_node->has_parameter("detector.min_confidence_threshold")) {
    shared_node->declare_parameter<double>("detector.min_confidence_threshold", 0.5);
  }
  shared_node->get_parameter("detector.min_confidence_threshold", min_confidence_threshold_);

  if (!shared_node->has_parameter("detector.iou_threshold")) {
    shared_node->declare_parameter<double>("detector.iou_threshold", 0.7);
  }
  shared_node->get_parameter("detector.iou_threshold", iou_threshold_);

  // Create service 'yolo_detection'
  callback_group_ = shared_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  executor_.add_callback_group(callback_group_, shared_node->get_node_base_interface());
  rclcpp::QoS qos = rclcpp::QoS(1);

  yolo_client_ = shared_node->create_client<sh_interfaces::srv::RunYoloDetection>(
    "yolo_detection",
    qos,
    callback_group_);

  if (!yolo_client_->wait_for_service(std::chrono::duration<double>(2.0))) {
    RCLCPP_INFO(shared_node->get_logger(), "Yolo Server not available in 2 seconds.");
    return false;
  }

  yolo_request_ = std::make_shared<sh_interfaces::srv::RunYoloDetection::Request>();

  return true;
}

void YoloDetectorPY::cleanup()
{
  executor_.remove_callback_group(callback_group_);
  callback_group_.reset();
  yolo_client_.reset();
  yolo_request_.reset();
}

bool YoloDetectorPY::detect(
  const cv::Mat& input,
  const sh_base_template::DetectorBaseConfig& detector_base_config,
  std::vector<sh_interfaces::msg::Detection2D>& detections)
{
  auto input_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    input).toImageMsg();

  yolo_request_->input = *input_msg;
  yolo_request_->yolo_config.gpu = detector_base_config.use_gpu;
  yolo_request_->yolo_config.input_size =
    {detector_base_config.input_size[0], detector_base_config.input_size[1]};
  yolo_request_->yolo_config.iou_threshold = iou_threshold_;
  yolo_request_->yolo_config.min_confidence_threshold = min_confidence_threshold_;
  yolo_request_->yolo_config.verbose = detector_base_config.inference_verbose;

  auto future = yolo_client_->async_send_request(yolo_request_);
  auto shared_node = node_.lock();
  if (executor_.spin_until_future_complete(future, std::chrono::duration<double>(2.0)) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto result = future.get();
  detections.resize(result->num_objects);
  detections = result->detections;
  return true;
}

}  // namespace sh_default_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_default_plugins::YoloDetectorPY, sh_base_template::DetectorBase)