#ifndef SH_BASE_TEMPLATE__STRUCTS__PERCEPTION_TYPES_HPP_
#define SH_BASE_TEMPLATE__STRUCTS__PERCEPTION_TYPES_HPP_

#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sh_base_template
{

namespace types
{

struct PerceptionFrame
{
  cv_bridge::CvImageConstPtr rgb;
  cv_bridge::CvImageConstPtr depth;
  sensor_msgs::msg::CameraInfo camera_info;
  sensor_msgs::msg::PointCloud2 point_cloud;
};

struct DetectorInput
{
  std::vector<PerceptionFrame> frames;
};

struct BoundingBox2D
{
  int x; // Centre x of the bbox
  int y; // Centre y of the bbox
  int width;
  int height;
};

struct DetectorResult
{
  BoundingBox2D bbox;
  int class_id;
  std::string class_name;
  double confidence;
  cv::Mat mask;
  std::string sensor_id; // Sensor id which the data is attached on
};

struct DetectorOutput
{
  std::vector<DetectorResult> output;
};

struct LocaliserInput
{
  PerceptionFrame frame;
  DetectorOutput detections;
};

struct PoseFeatures
{
  geometry_msgs::msg::Pose pose;
  bool valid_pose;
};

struct LocaliserOutput
{
  std::vector<PoseFeatures> poses;
};

}  // namespace types

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__STRUCTS__PERCEPTION_TYPES_HPP_