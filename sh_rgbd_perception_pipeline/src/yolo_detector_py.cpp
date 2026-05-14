#include "sh_rgbd_perception_pipeline/yolo_detector_py.hpp"

#include <chrono>

// #include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "sh_utils/ros2_node_utils.hpp"

namespace sh_rgbd_perception_pipeline
{

bool YoloDetectorPY::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  node_ = node;
  auto shared_node = node_.lock();

  sh_utils::ros2_node_utils::declare_and_get_parameter<bool>(
    shared_node, "pipeline.detector.inference_verbose",
    detector_config_.inference_verbose, true);

  sh_utils::ros2_node_utils::declare_and_get_parameter<bool>(
    shared_node, "pipeline.detector.use_gpu",
    detector_config_.use_gpu, true);

  sh_utils::ros2_node_utils::declare_and_get_parameter<double>(
    shared_node, "pipeline.detector.min_confidence_threshold",
    detector_config_.min_confidence_threshold, 0.7);

  sh_utils::ros2_node_utils::declare_and_get_parameter<double>(
    shared_node, "pipeline.detector.iou_threshold",
    detector_config_.iou_threshold, 0.7);

  // std::string current_pkg = ament_index_cpp::get_package_share_directory("sh_rgbd_perception_pipeline");
  // sh_utils::declare_and_get_parameter<std::string>(
  //   shared_node, "pipeline.detector.model_path",
  //   detector_config_.model_path, current_pkg+"/model/apple_mobi.pt");

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<int64_t>>(
    shared_node, "pipeline.detector.input_size",
    detector_config_.input_size, {1280, 736});

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
    RCLCPP_ERROR(shared_node->get_logger(), "Yolo Server not available in 2 seconds.");
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
  const sh_base_template::types::DetectorInput& detector_input,
  sh_base_template::types::DetectorOutput& detector_output)
{
  if (!detector_input.frames.size()) {
    RCLCPP_ERROR(
      node_.lock()->get_logger(),
      "Empty input of type [sh_base_template::types::DetectorInput] provided");
    return false;
  }

  // ONLY PROCESS THE FIRST RGB IMAGE AT THE MOMENT

  auto input_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    "bgr8",
    detector_input.frames[0].rgb->image).toImageMsg();

  yolo_request_->input = *input_msg;
  yolo_request_->yolo_config.gpu = detector_config_.use_gpu;
  yolo_request_->yolo_config.input_size =
    {detector_config_.input_size[0], detector_config_.input_size[1]};
  yolo_request_->yolo_config.iou_threshold = detector_config_.iou_threshold;
  yolo_request_->yolo_config.min_confidence_threshold = detector_config_.min_confidence_threshold;
  yolo_request_->yolo_config.verbose = detector_config_.inference_verbose;

  auto future = yolo_client_->async_send_request(yolo_request_);
  auto shared_node = node_.lock();
  if (
    executor_.spin_until_future_complete(future, std::chrono::duration<double>(2.0)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_.lock()->get_logger(),
      "No reponse received in 2 seconds from YoloPY server");
    return false;
  }

  auto result = future.get();
  detector_output.output.resize(result->num_objects);


  for (int i = 0; i < result->num_objects; ++i)
  {
    // Centre (x,y) of the bbox with size: width x height
    detector_output.output[i].bbox.x = (int)((result->detections[i].x2+result->detections[i].x1)/2);
    detector_output.output[i].bbox.y = (int)((result->detections[i].y2+result->detections[i].y1)/2);
    detector_output.output[i].bbox.width = result->detections[i].x2-result->detections[i].x1;
    detector_output.output[i].bbox.height = result->detections[i].y2-result->detections[i].y1;

    detector_output.output[i].class_id = result->detections[i].class_id;
    detector_output.output[i].class_name = result->detections[i].class_name;
    detector_output.output[i].confidence = result->detections[i].confidence;
    detector_output.output[i].sensor_id = detector_input.frames[0].rgb->header.frame_id;
  }

  return true;
}

}  // namespace sh_rgbd_perception_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_rgbd_perception_pipeline::YoloDetectorPY, sh_base_template::DetectorBase)