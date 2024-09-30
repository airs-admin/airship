import logging
import time
import threading

from cv_bridge import CvBridge
from rclpy.node import Node
import cv2
import numpy as np
import rclpy

from airship_interface.srv import SegmentationMapping
from lib.config import Config

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')
DEFAULT_RESULT = ([], [], [], [])

class SegmentationServiceClient(Node):
    def __init__(self, config_path, node_name='segmentation_service_clinet', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.config = Config(config_path)
        self.path_label_table = self.config.get('path_label_table', '/home/airsbot2/airship/src/airship/airship_perception/lib/configs/semantic_label_table.txt')
        self.request_timeout = self.config.get('request_timeout', 15.0)
        self.name_segmentation_service_server = self.config.get('name_segmentation_service_server', '/airship_perception/SegmentationMapping')

        self.cv_bridge = CvBridge()
        self.label_to_instance = self.load_labels(self.path_label_table)
        self.segmentation_service_client = self.create_client(SegmentationMapping, self.name_segmentation_service_server)

    def find_instance(self, labels):
        return [self.label_to_instance.get(label, -1) for label in labels]

    def load_labels(self, label_file_path):
        label_to_instance = {}
        try:
            with open(label_file_path, 'r') as file:
                for index, line in enumerate(file):
                    label = line.strip()
                    if label:
                        label_to_instance[label] = index + 1
        except FileNotFoundError:
            logging.error(f'Label file {label_file_path} not found.')
        return label_to_instance

    def process_masks(self, masks_msgs):
        masks = []
        for mask_msg in masks_msgs:
            # Convert each ROS Image message to a NumPy array
            mask_cv = self.cv_bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
            # Ensure mask is binary (0 and 1)
            _, binary_mask = cv2.threshold(mask_cv, 1, 1, cv2.THRESH_BINARY)
            masks.append(binary_mask.astype(np.uint8))
        return masks

    def request_segmentation_service(self, image):
        request = SegmentationMapping.Request()
        request.image = image
        response = self.segmentation_service_client.call_async(request)
        # Check timeout
        start_time = time.time()
        while not response.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (time.time() - start_time)
            if elapsed_time > self.request_timeout:
                logging.error(f'Service call to {self.service_name} timed out after {elapsed_time:.2f} seconds.')
                return DEFAULT_RESULT
        # Recieved service response
        if response.done():
            result = response.result()
            if result is not None:
                logging.info('Response received.')
                masks_numpy = self.process_masks(result.masks)
                instances = self.find_instance(result.labels)
                # Todo: remove the masks with instance < 0
                visualized_output = self.visualize_masks(image, masks_numpy, instances)
                return instances, masks_numpy, result.labels, visualized_output
            else:
                logging.error('Response is empty.')
                return DEFAULT_RESULT

    def run(self, image):
        result = {}
        def target():
            result['data'] = self.run_thread(image)
        thread = threading.Thread(target=target)
        thread.start()
        thread.join()
        return result.get('data', DEFAULT_RESULT)

    def run_thread(self, image):
        instances, masks, labels, visualized_output = self.request_segmentation_service(image)
        masks = np.array(masks)
        labels = np.array(labels)
        instances = np.array(instances)
        return instances, labels, masks, visualized_output

    def visualize_masks(self, image, masks, instances):
        original_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        output_image = original_image.copy()
        colors = [[instance % 256, (instance * 2) % 256, (instance * 3) % 256] for instance in instances]
        for mask, color in zip(masks, colors):
            for c in range(3):
                output_image[:, :, c] = np.where(mask > 0, mask * color[c], output_image[:, :, c])
        return output_image
