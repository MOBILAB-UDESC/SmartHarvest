from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import (
    LifecycleNode,
    LifecycleState,
    TransitionCallbackReturn
)
from sensor_msgs.msg import (
    CameraInfo,
    Image
)
import supervision as sv
from time import sleep
from ultralytics import YOLO

from sh_detection_server_py.parameter_handler import ParameterHandler
from sh_interfaces.msg import (
    DetectedObjects,
    SingleObjectInfo
)
from sh_interfaces.srv import DetectObjects


class DetectionServer(LifecycleNode):
    """
        Lifecycle Node for object detection.

        This node provides a service "get_detections" and uses YOLO for object detection.
        The node combines RGB + depth data to estimate 3D positions.
    """

    def __init__(self):
        super().__init__('sh_detection_server')

        self.model = None
        self.device = None
        self.bridge = None
        self.bounding_box_annotator = None
        self.service = None
        self.image_publisher = None
        self.param_handler = None
        self.parameters = None
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        self.get_logger().info(f'Lifecycle {self.get_name()} created.')

    def on_configure(self, state: LifecycleState):
        """
            Callback function for configure transition.

            Declare and retrieve parameters.
            Load YOLO model.

            @param state Current lifecycle state.
            @return Transition callback return status.
        """
        self.get_logger().info('Configuring...')

        self.param_handler = ParameterHandler(self)
        self.parameters = self.param_handler.get_parameters()
        self.get_logger().info('Parameters loaded.')

        # Images/model configuration
        self.bridge = CvBridge()
        self.bounding_box_annotator = sv.BoxAnnotator()
        try:
            self.model = YOLO(self.parameters.model_path, task='detect')
            self.get_logger().info(f'Model loaded from: {self.parameters.model_path}.')
            self.detection_warmup()
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load model: {e}.')
            self.get_logger().error('❌ Ensure the path is correct (absolute or package-relative).')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('✅ Lifecycle Node configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        """
            Callback function for activate transition.

            Create detection service.
            Create image publisher if enabled.

            @param state Current lifecycle state.
            @return Transition callback return status.
        """
        self.get_logger().info('Activating...')

        self.service = self.create_service(
            DetectObjects,
            'get_detections',
            self.trigger_detections_callback
        )

        if self.parameters.publish_detections_image:
            self.image_publisher = self.create_publisher(Image, 'detection_image', 10)

        self.get_logger().info('✅ Lifecycle Node activated - ready for calls.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        """
            Callback function for deactivate transition.

            Destroy detection service and image publisher if exists.

            @param state Current lifecycle state.
            @return Transition callback return status.
        """
        self.get_logger().info('Deactivating...')

        # Destroy service
        self.destroy_service(self.service)
        # Destroy subscriber if enabled
        if self.parameters.publish_detections_image:
            self.destroy_publisher(self.image_publisher)

        self.get_logger().info('✅ Lifecycle Node deactivated.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        """
            Callback function for clean up transition.

            Release resources.

            @param state Current lifecycle state.
            @return Transition callback return status.
        """
        self.get_logger().info('Cleaning up...')

        self.model = None
        self.device = None
        self.bridge = None
        self.bounding_box_annotator = None

        if self.parameters.visualize_rgb or self.parameters.visualize_depth:
            cv2.destroyAllWindows()

        self.parameters = None
        self.param_handler = None

        self.get_logger().info('✅ Lifecycle Node cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        """
            Callback function for shutdown transition.

            @param state Current lifecycle state.
            @return Transition callback return status.
        """
        self.get_logger().info('Shutting down...')

        if state.label != 'cleanup':
            self.on_cleanup(state)

        self.get_logger().info('✅ Lifecycle Node shutted down.')
        return TransitionCallbackReturn.SUCCESS

    def detection_warmup(self):
        """
            Warm up the YOLO model with dummy images.
        """

        self.get_logger().info('Starting model warm-up...')

        warmup_image = np.random.randint(
            0, 256,
            (self.parameters.input_size[0], self.parameters.input_size[1], 3),
            dtype=np.uint8
        )

        for i in range(5):
            result = self.model.predict(
                source=warmup_image,
                conf=self.parameters.min_inference_confidence,
                save=False,
                imgsz=(self.parameters.input_size[0], self.parameters.input_size[1]),
                verbose=False
            )
            sleep(0.100)

        self.get_logger().info('✅ Model warm-up complete')

    def trigger_detections_callback(self, request, response):
        """
            Handle detection service requests.

            Process RGBD images to detect objects and estimate their 3D positions.

            @param request Service request containing RGB image, depth image, and camera info.
            @param response Service response to populate with detection results.

            @return response Populated service response with detected objects.
        """
        self.get_logger().info('Starting detection')
        rgb_image = request.rgb_image
        depth_image = request.depth_image
        self.depth_encoding = depth_image.encoding
        cam_info = request.cam_info

        rgb_image_cv = self.bridge.imgmsg_to_cv2(rgb_image, 'bgr8')
        depth_image_cv = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

        if self.parameters.use_gpu:
            self.device = 'cuda:0'
        else:
            self.device = 'cpu'

        results = self.model.predict(
            source=rgb_image_cv,
            conf=self.parameters.min_inference_confidence,
            save=False,
            device=self.device,
            imgsz=(self.parameters.input_size[0], self.parameters.input_size[1])
        )[0]

        # Annotate image
        detections = sv.Detections.from_ultralytics(results)
        rgb_image_cv = self.bounding_box_annotator.annotate(
            scene=rgb_image_cv, detections=detections)

        response.success = True
        response.detected_objects.header = cam_info.header
        response.detected_objects.num_objects = len(detections)
        response.detected_objects.objects = self.upgrade_poses(
            detections,
            depth_image_cv,
            cam_info.k
        )

        if self.parameters.publish_detections_image:
            self.publish_image(rgb_image_cv)

        # Show images if visualization is enabled
        self.show_image(rgb_image_cv, depth_image_cv)

        return response

    def upgrade_poses(self, detections, depth_image, camera_intrinsics):
        """
            Get 3D position and inference metrics of detected objects.

            @param detections Supervision Detections object with bounding boxes.
            @param depth_image Depth image.
            @param camera_intrinsics Camera intrinsic matrix.
            @return detected_objects List of detected objects with 3D positions.
        """
        fx = camera_intrinsics[0]
        fy = camera_intrinsics[4]
        cx = camera_intrinsics[2]
        cy = camera_intrinsics[5]

        detected_objects = []

        for bbox, conf, class_id in zip(
                detections.xyxy, detections.confidence, detections.class_id):
            object_info = SingleObjectInfo()
            [x, y, z] = self.get_position(depth_image, bbox, fx, fy, cx, cy)
            [roll, pitch, yaw] = self.get_orientation([x, y, z])
            object_info.x = x
            object_info.y = y
            object_info.z = z
            object_info.grasp_orientation_euler[0] = roll
            object_info.grasp_orientation_euler[1] = pitch
            object_info.grasp_orientation_euler[2] = yaw
            object_info.confidence = float(conf)
            object_info.class_id = int(class_id)
            if self.use_sim_time:
                object_info.distance = object_info.x
            else:
                object_info.distance = object_info.z
            detected_objects.append(object_info)

        return detected_objects

    def get_position(self, depth_image, bbox, fx, fy, cx, cy):
        """
            Estimate the 3D position of an object from its bounding box and depth information.
            Reference: https://www.mdpi.com/2073-4395/13/7/1816

            @param depth_image Depth image.
            @param bbox Bounding box [x1, y1, x2, y2]
            @param fx Focal length in x direction.
            @param fy Focal length in y direction.
            @param cx Principal point x coordinate.
            @param cy Principal point y coordinate
            @return detected_objects List of 3D position in meters.
        """
        # Get the depth value at the center and adjacents points of the bounding box
        x_center = int((bbox[0] + bbox[2]) / 2)
        y_center = int((bbox[1] + bbox[3]) / 2)
        points = [
            (x_center, y_center),
            (x_center - 1, y_center),
            (x_center + 1, y_center),
            (x_center, y_center - 1),
            (x_center, y_center + 1),
        ]

        # Depth mean value from the points
        depth_scale = self.get_depth_scale(self.depth_encoding)
        depths = [depth_image[y, x] * depth_scale for x, y in points if depth_image[y, x] > 0]
        z0 = sum(depths) / len(depths) if len(depths) > 0 else 0.0

        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]

        # Estimate sphere radius if treating object as spherical (e.g., apples)
        r = (z0 * w / (2 * fx)) if w >= h else (z0 * h / (2 * fy))

        X = z0 * (x_center - cx) / fx
        Y = z0 * (y_center - cy) / fy
        Z = z0 + r

        # Simulation and real camera frames differ, so axis mapping is adjusted here.
        if self.use_sim_time:
            self.get_logger().info(f'USE_SIM_TIME: {self.use_sim_time}')
            return [Z, -X, -Y]
        else:
            self.get_logger().info(f'USE_SIM_TIME: {self.use_sim_time}')
            return [X, Y, Z]

    def get_orientation(self, position):
        """
            Compute grasp orientation angles roll, pitch, yaw from object position in camera frame.

            Returns:
                list: [roll, pitch, yaw] euler angles

            @param position 3D position of an object.
        """
        x = position[0]
        y = position[1]
        z = position[2]

        if self.use_sim_time:
            roll = np.arctan2(z, x)
            yaw = np.arctan2(y, x)
            pitch = 0.0
        else:
            roll = np.arctan2(x, z)
            yaw = 0.0
            pitch = np.arctan2(-y, np.sqrt(x**2 + z**2))

        return [roll, pitch, yaw]

    def get_depth_scale(self, encoding):
        """
            Determine the depth scale factor based on image encoding.

            @param encoding Depth image encoding.
            @return Depth scale factor to convert depth values to meters.
        """
        if encoding == '16UC1':
            return 0.001  # mm to m
        elif encoding == '32FC1':
            return 1.0  # m
        else:
            self.get_logger().warn(f'Unknown depth encoding: {encoding}, assuming meters')
            return 1.0

    def publish_image(self, rgb_image):
        """
            Publish the annotated RGB image.

            @param rgb_image RGB image.
        """
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
        self.image_publisher.publish(rgb_msg)

    def show_image(self, rgb_image, depth_image):
        """
            Display RGB and depth images using OpenCV windows.

            @param rgb_image RGB image.
            @param depth_image Depth image.
        """
        if self.parameters.visualize_rgb:
            cv2.imshow('RGB', rgb_image)

        if self.parameters.visualize_depth:
            depth_image = cv2.normalize(
                depth_image,
                None,
                0,
                255,
                cv2.NORM_MINMAX,
                cv2.CV_8U
            )

            cv2.imshow('Depth', depth_image)


def main(args=None):
    rclpy.init(args=args)

    node = DetectionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
