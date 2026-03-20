from ament_index_python.packages import get_package_share_directory
from dataclasses import dataclass
import rclpy
from rclpy.parameter import Parameter
from rclpy.lifecycle import LifecycleNode
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


@dataclass(init=False)
class Parameters:
    model_path: str = ''
    input_size: list[int] = None
    use_gpu: bool = True
    min_inference_confidence: float = 0.0
    inference_iou_threshold: float = 0.0
    inference_max_objects: int = 0
    visualize_rgb: bool = False
    visualize_depth: bool = False
    publish_detections_image: bool = True


class ParameterHandler:
    def __init__(self, node: LifecycleNode):
        self.node = node
        self.parameters = Parameters()

        self.declare_parameters()

        self.node.add_on_set_parameters_callback(self.dynamic_parameters_callback)

    def declare_parameters(self):
        self.parameters.model_path = self.declare_parameter_if_not_declared(
            'model_path',
            'model/apple.pt',
            'Path to the YOLO detection model file (.pt). Can be absolute or relative to workspace')
        # Get absolute path if package-relative is given
        if not self.parameters.model_path.startswith('/'):
            shared_pkg = get_package_share_directory('sh_detection_server_py')
            self.parameters.model_path = f'{shared_pkg}/{self.parameters.model_path}'
        self.parameters.input_size = self.declare_parameter_if_not_declared(
            'input_size',
            [640, 640],
            'Input image dimensions [width, height] in pixels')
        self.parameters.use_gpu = self.declare_parameter_if_not_declared(
            'use_gpu',
            True,
            'Wheter to use gpu or not for inference')
        self.parameters.min_inference_confidence = self.declare_parameter_if_not_declared(
            'min_inference_confidence',
            0.45,
            'Minimum confidence threshold for detected objects to be considered valid')
        self.parameters.inference_iou_threshold = self.declare_parameter_if_not_declared(
            'inference_iou_threshold',
            0.25,
            'Minimum intersection over union threshold for detected objects to be considered valid')
        self.parameters.inference_max_objects = self.declare_parameter_if_not_declared(
            'inference_max_objects',
            25,
            'Maximum number of objects to detect per frame')
        self.parameters.visualize_rgb = self.declare_parameter_if_not_declared(
            'visualize_rgb',
            False,
            'Display RGB image with bounding boxes using OpenCV. May reduce performance')
        self.parameters.visualize_depth = self.declare_parameter_if_not_declared(
            'visualize_depth',
            False,
            'Display depth image with bounding boxes using OpenCV. May reduce performance')
        self.parameters.publish_detections_image = self.declare_parameter_if_not_declared(
            'publish_detections_image',
            False,
            'Publish anotated RGB image with detection bounding boxes to a ROS 2 topic')

    def declare_parameter_if_not_declared(self, parameter: str, default_value, description: str):
        if not self.node.has_parameter(parameter):
            param_descriptor = ParameterDescriptor(description=description)
            self.node.declare_parameter(parameter, default_value, param_descriptor)

        return self.load_parameter(parameter)

    def get_parameters(self):
        return self.parameters

    def load_parameter(self, parameter: str):
        param = self.node.get_parameter(parameter)
        type = param.type_
        if type == rclpy.Parameter.Type.BOOL:
            return param.get_parameter_value().bool_value
        if type == rclpy.Parameter.Type.DOUBLE:
            return param.get_parameter_value().double_value
        if type == rclpy.Parameter.Type.INTEGER:
            return param.get_parameter_value().integer_value
        if type == rclpy.Parameter.Type.INTEGER_ARRAY:
            return param.get_parameter_value().integer_array_value
        if type == rclpy.Parameter.Type.STRING:
            return param.get_parameter_value().string_value

    def dynamic_parameters_callback(self, params: list[Parameter]):
        success = True
        for param in params:
            type = param.type_
            name = param.name
            if type == rclpy.Parameter.Type.BOOL:
                if name == 'use_gpu':
                    self.parameters.use_gpu = param.value
                if name == 'visualize_rgb':
                    self.parameters.visualize_rgb = param.value
                if name == 'visualize_depth':
                    self.parameters.visualize_depth = param.value
                if name == 'publish_detections_image':
                    self.parameters.publish_detections_image = param.value
            elif type == rclpy.Parameter.Type.DOUBLE:
                if name == 'min_inference_confidence':
                    self.parameters.min_inference_confidence = param.value
                if name == 'inference_iou_threshold':
                    self.parameters.inference_iou_threshold = param.value
            elif type == rclpy.Parameter.Type.INTEGER:
                if name == 'inference_max_objects':
                    self.parameters.inference_max_objects = param.value
            elif type == rclpy.Parameter.Type.INTEGER_ARRAY:
                if name == 'input_size':
                    self.parameters.input_size = param.value
            elif type == rclpy.Parameter.Type.STRING:
                if name == 'model_path':
                    self.parameters.model_path = param.value
                    self.node.get_logger().warn(
                        'model_path parameter has been updated. '
                        'Reconfigure the lifecycle node to load the new model.')

        return SetParametersResult(successful=success)
