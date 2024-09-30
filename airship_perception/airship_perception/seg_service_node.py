import yaml

from cv_bridge import CvBridge
from groundingdino.util.inference import Model
from rclpy.node import Node
import rclpy
import torch

from airship_interface.srv import SegmentationGrasping, SegmentationMapping
from lib.configs import set_config
from lib.grounded_sam_api import grounded_sam
from lib.segment_anything.segment_anything import SamPredictor, sam_model_registry

class SegmentationServiceServer(Node):
    def __init__(self, node_name="airship_perception_server_node"):
        super().__init__(node_name=node_name)
        self.get_logger().info("airship_perception_server_node initialization process started.")
        self.cv_bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Set paths
        self.declare_parameter('config', 'config.yaml')
        self.config_path = self.get_parameter('config').get_parameter_value().string_value
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        self.path_config_grounding_dino = self.config.get('path_config_grounding_dino')
        self.path_model_bert_base_uncased = self.config.get('path_model_bert_base_uncased')
        self.path_model_grounding_dino = self.config.get('path_model_grounding_dino')
        self.path_model_sam = self.config.get('path_model_sam')
        self.path_map_classes = self.config.get('path_map_classes')

        # Services
        self.seg2grasp = self.create_service(SegmentationGrasping, 'airship_perception/SegmentationGrasping', self.segmentation_grasp_callback)
        self.seg2map = self.create_service(SegmentationMapping, 'airship_perception/SegmentationMapping', self.segmentation_mapping_callback)

        # GroundingDINO model
        set_config.reset_cfg(self.path_config_grounding_dino, 'text_encoder_type', self.path_model_bert_base_uncased)
        self.grounding_dino_model = Model(model_config_path=self.path_config_grounding_dino, model_checkpoint_path=self.path_model_grounding_dino)

        # SAM model
        self.sam_encoder_version = "vit_h"
        sam = sam_model_registry[self.sam_encoder_version](checkpoint=self.path_model_sam)
        sam.to(device=self.device)
        self.sam_predictor = SamPredictor(sam)

        # Map classes
        with open(self.path_map_classes, 'r', encoding='utf-8') as file:
            self.map_classes = [line.strip() for line in file]

        self.get_logger().info("airship_perception_server_node initialization process done.")

    def segmentation_grasp_callback(self, request, response):
        # RGB image
        rgb_image = request.image
        rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
        # masks
        object_list = [request.object]
        masks, labels = grounded_sam(self.grounding_dino_model, self.sam_predictor, rgb_image, object_list, box_threshold=0.5, text_threshold=0.4, nms_threshold=0.8, action='grasp')
        # response
        response.mask = self.cv_bridge.cv2_to_imgmsg(masks[0], encoding = "mono8")
        return response

    def segmentation_mapping_callback(self, request, response):
        # RGB image
        rgb_image = request.image
        rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image, desired_encoding='bgr8')
        # masks
        object_list = self.map_classes
        masks, labels = grounded_sam(self.grounding_dino_model, self.sam_predictor, rgb_image, object_list, box_threshold=0.3, text_threshold=0.25, nms_threshold=0.8, action='map')
        ros_masks = []
        for mask in masks:
            mask = self.cv_bridge.cv2_to_imgmsg(mask, encoding="mono8")
            ros_masks.append(mask)
        # response
        response.masks = ros_masks
        response.labels = labels
        return response

def main(args=None):
    rclpy.init(args=args)
    server_node = SegmentationServiceServer("airship_perception_server_node")

    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()