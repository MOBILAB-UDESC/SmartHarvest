#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from cv_bridge import (
    CvBridge,
    CvBridgeError
)
import numpy as np
import rclpy
from rclpy.node import Node
import supervision as sv
from time import sleep
from ultralytics import YOLO

from sh_interfaces.msg import Detection2D
from sh_interfaces.srv import RunYoloDetection


class YoloServer(Node):
    """
    ROS 2 node that provides a service 'yolo_detection' and uses YOLO for 2D object detection.
    """

    def __init__(self):
        super().__init__('yolo_server')
        self.get_logger().info('Initializing')

        self.bridge = CvBridge()
        self.bounding_box_annotator = sv.BoxAnnotator()

        if not self.has_parameter('model_path'):
            model_pkg = get_package_share_directory('sh_rgbd_perception_pipeline')
            self.declare_parameter('model_path', f'{model_pkg}/model/apple_mobi.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info(f'Model loaded from: {model_path}.')
            self.detection_warmup()
            self.class_names = self.model.names
        except Exception as e:
            self.get_logger().fatal(f'Failed to load model: {e}.')
            raise RuntimeError('Unable to initialize YOLO model')

        self.service = self.create_service(srv_type=RunYoloDetection,
                                           srv_name='yolo_detection',
                                           callback=self.yolo_detection_callback)

        self.get_logger().info('Initialized')

    def yolo_detection_callback(
            self,
            request: RunYoloDetection.Request,
            response: RunYoloDetection.Response):
        """
        Handle detection service requests.

        :param request Service request containing RGB image and inference configs.
        :param response Service response to populate with detection results.
        :return: response.
        """
        try:
            rgb_image_cv = self.bridge.imgmsg_to_cv2(request.input, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'{e}')
            response.success = False
            return response

        device = 'cpu'
        if request.yolo_config.gpu:
            device = 'cuda:0'

        results = self.model.predict(
            source=rgb_image_cv,
            save=False,
            device=device,
            conf=request.yolo_config.min_confidence_threshold,
            iou=request.yolo_config.iou_threshold,
            imgsz=[request.yolo_config.input_size[0], request.yolo_config.input_size[1]],
            verbose=request.yolo_config.verbose,
        )[0]

        detections = sv.Detections.from_ultralytics(results)

        detected_objects = []
        for bbox, conf, class_id in zip(
                detections.xyxy, detections.confidence, detections.class_id):

            detection = Detection2D()
            detection.x1 = int(bbox[0])
            detection.y1 = int(bbox[1])
            detection.x2 = int(bbox[2])
            detection.y2 = int(bbox[3])
            detection.confidence = float(conf)
            detection.class_id = int(class_id)
            detection.class_name = self.class_names[class_id]
            detected_objects.append(detection)

        response.success = True
        response.num_objects = int(len(detected_objects))
        response.detections = detected_objects

        return response

    def detection_warmup(self):
        """
        Model warmup with dummy images.
        """

        self.get_logger().info('Starting warmup.')

        warmup_image = np.random.randint(
            0, 256,
            (1280, 736, 3),
            dtype=np.uint8
        )

        for i in range(5):
            self.model.predict(
                source=warmup_image,
                conf=0.5,
                save=False,
                imgsz=(1280, 736),
                verbose=False
            )
            sleep(0.100)

        self.get_logger().info('Model warmup complete.')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass


if __name__ == "__main__":
    main()
