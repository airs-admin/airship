import time
import threading

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
import message_filters
import rclpy

from airship_interface.srv import SaveObjectNavGoal
from lib.config import Config
from lib.io_nav_goal import IONavGoal
from lib.map import Map
from lib.map_2d import Map2D
from lib.object_depth import ObjectDepth
from lib.plot_ros import PlotROS
from lib.segmentation_service_client import SegmentationServiceClient
from lib.utility_ros import UtilityROS

class ObjectMap(Node):
    def __init__(self):
        super().__init__('object_map')
        # Declare and get parameter
        self.declare_parameter('config', 'config.yaml')
        path_config = self.get_parameter('config').get_parameter_value().string_value
        self.config = Config(path_config)
        self.path_config_object_depth = self.config.get('path_config_object_depth', 'config_object_depth.yaml')
        self.path_config_segmentation_service_client = self.config.get('path_config_segmentation_service_client', 'config_segmentation_service_client.yaml')        
        # Topics of subscriber
        self.topic_sub_depth = self.config.get('topic_sub_depth', '/zed/zed_node/depth/depth_registered')
        self.topic_sub_image = self.config.get('topic_sub_image', '/zed/zed_node/left_gray/image_rect_gray')
        self.topic_sub_odometry = self.config.get('topic_sub_odometry', '/tracked_pose')
        # Topics of publisher
        self.topic_pub_keyframe_cloud = self.config.get('topic_pub_keyframe_cloud', '/airship_object/keyframe_cloud')
        self.topic_pub_keyframe_marker = self.config.get('topic_pub_keyframe_marker', '/airship_object/keyframe_marker')
        self.topic_pub_object_marker = self.config.get('topic_pub_object_marker', '/airship_object/object_marker')
        self.topic_pub_object_nav_goal_marker = self.config.get('topic_pub_object_nav_goal_marker', '/airship_object/object_nav_goal_marker')
        self.topic_pub_object_segmentation = self.config.get('topic_pub_object_segmentation', '/airship_object/object_segmentation')
        # Message type of odometry
        message_type = {'PoseStamped': PoseStamped, 'Odometry': Odometry}
        self.message_type_odomtry = message_type.get(self.config.get('message_type_odomtry', 'PoseStamped'))
        if not self.message_type_odomtry:
            raise ValueError('Unsupported message type.')
        # Initialize subscriber
        self.sub_depth = message_filters.Subscriber(self, Image, self.topic_sub_depth)
        self.sub_image = message_filters.Subscriber(self, Image, self.topic_sub_image)
        self.sub_odometry = message_filters.Subscriber(self, self.message_type_odomtry, self.topic_sub_odometry)
        # Initialize publisher
        self.pub_keyframe_cloud = self.create_publisher(PointCloud2, self.topic_pub_keyframe_cloud, 10)
        self.pub_keyframe_marker = self.create_publisher(MarkerArray, self.topic_pub_keyframe_marker, 10)
        self.pub_object_marker = self.create_publisher(MarkerArray, self.topic_pub_object_marker, 10)
        self.pub_object_nav_goal_marker = self.create_publisher(MarkerArray, self.topic_pub_object_nav_goal_marker, 10)
        self.pub_object_segmentation = self.create_publisher(Image, self.topic_pub_object_segmentation, 10)
        # Message synchronizer
        self.ats = message_filters.ApproximateTimeSynchronizer([self.sub_depth, self.sub_image, self.sub_odometry], 50, 0.1)
        self.ats.registerCallback(self.callback_message)
        # Object goal service server
        self.srv_save_object_nav_goal = self.create_service(SaveObjectNavGoal, '/airship_object/save_object_nav_goal', self.callback_service_save_object_nav_goal)
        # Other objects
        self.cv_bridge = CvBridge()
        self.io_nav_goal = IONavGoal()
        self.map = Map()
        self.map_2d = Map2D()
        self.object_depth = ObjectDepth(self.path_config_object_depth)
        self.plot_ros = PlotROS()
        self.segmentation_service_client = SegmentationServiceClient(self.path_config_segmentation_service_client)
        self.utility_ros = UtilityROS()
        # Thread of visualization and cluster
        self.thread_cluster_object_cloud = threading.Thread(target=self.thread_cluster_object_cloud)
        self.thread_cluster_object_cloud.daemon = True
        self.thread_vis_object_map = threading.Thread(target=self.thread_vis_object_map)
        self.thread_vis_object_map.daemon = True
        self.lock = threading.Lock()
        self.thread_cluster_object_cloud.start()
        self.thread_vis_object_map.start()

    def callback_message(self, msg_depth, msg_image, msg_odometry):
        # If the lock is already held, we skip this callback invocation
        if self.lock.locked():
            return
        with self.lock:
            # Run segmentation
            object_instances, object_labels, object_masks, object_segmentation_image = self.segmentation_service_client.run(msg_image)
            if len(object_labels) > 0:
                depth = self.utility_ros.depth_message_to_mat(msg_depth)
                image = self.utility_ros.image_message_to_mat(msg_image)
                # Publish segmentation image
                msg_segmentation_image = self.cv_bridge.cv2_to_imgmsg(object_segmentation_image, encoding='rgb8')
                self.pub_object_segmentation.publish(msg_segmentation_image)
                # Add new keyframes
                object_clouds, valid_indices = self.object_depth.get_clouds_by_masks(object_masks, depth, image)
                if len(valid_indices) > 0:
                    keyframe = self.create_keyframe(msg_odometry, depth, image, object_clouds, object_instances[valid_indices], object_labels[valid_indices], object_masks[valid_indices])
                    self.map.append_keyframe(keyframe)

    def callback_service_save_object_nav_goal(self, request, response):
        self.get_logger().info(f'Save object navigation goal to {request.path_nav_goal_list}')
        if self.map.get_object_size() == 0:
            return self.create_response(response, "There is no object.", False)
        # Get objects and find object navigation goal.
        objects = self.map.get_objects()
        list_pose_2d = self.map.project_2d(objects)
        list_pose_2d_w_heading, list_object_label = self.map_2d.find_nav_pose_based_on_observe_pose(list_pose_2d, objects)
        # Write to yaml
        yaml_object = self.io_nav_goal.create_objects(list_object_label, list_pose_2d_w_heading)
        self.io_nav_goal.write_nav_goal_to_yaml(yaml_object, request.path_nav_goal_list)
        # Publish object navigation goal.
        marker = self.plot_ros.create_object_nav_goal_marker(list_pose_2d_w_heading, self.get_clock().now().to_msg())
        self.pub_object_nav_goal_marker.publish(marker)
        return self.create_response(response, "Success.", True)

    def create_keyframe(self, msg_odometry, depth, image, object_clouds, object_instances, object_labels, object_masks):
        timestamp = self.utility_ros.get_time_from_header(msg_odometry.header)
        position, orientation = self.map_message_type_to_function(msg_odometry)
        for i, cloud in enumerate(object_clouds):
            object_clouds[i] = self.map.downsample_cloud(cloud)
        object_clouds_global = self.map.convert_to_global_cloud(object_clouds, orientation, position)
        keyframe = self.map.create_keyframe(timestamp=timestamp, depth=depth, image=image, 
                                             object_clouds=object_clouds, object_clouds_global=object_clouds_global,
                                             object_instances=object_instances, object_labels=object_labels, object_masks=object_masks,
                                             orientation=orientation, position=position)
        return keyframe
    
    def create_response(self, response, message, result):
        response.message = message
        response.result = result
        self.get_logger().info(f'Response: {message}, Result: {result}')
        return response

    def map_message_type_to_function(self, msg_odometry):
        message_type_to_function = {PoseStamped: self.utility_ros.pose_stamped_message_to_position_and_orientation,
                                    Odometry: self.utility_ros.odometry_message_to_position_and_orientation}
        position, orientation = message_type_to_function[self.message_type_odomtry](msg_odometry)
        return position, orientation

    def thread_cluster_object_cloud(self):
        while rclpy.ok():
            if self.map.is_keyframe_updated() is False:
                continue
            keyframes = self.map.get_keyframes()
            cloud, list_cloud_instance, list_pcd_label, list_pose_orientation, list_pose_position, list_cloud_size = self.map.merge_clouds(keyframes)
            if len(cloud.points) > 0:
                objects = self.map.cluster_cloud(cloud, list_cloud_instance, list_pcd_label, list_pose_orientation, list_pose_position, list_cloud_size)
                self.map.update_objects(objects)
            time.sleep(1.0)

    def thread_vis_object_map(self):
        while rclpy.ok():
            # Publish keyframe bouding box 
            if self.map.get_keyframe_size() == 0:
                continue
            keyframes = self.map.get_keyframes()
            list_label, list_aabb = self.map.get_semantic_bouding_box(keyframes)
            marker = self.plot_ros.create_semantic_bouding_box_with_label_marker(list_label, list_aabb, self.get_clock().now().to_msg())
            self.pub_keyframe_marker.publish(marker)
            # Publish object bouding box 
            if self.map.get_object_size() == 0:
                continue
            objects = self.map.get_objects()
            list_label, list_aabb = self.map.get_semantic_bouding_box(objects)
            marker = self.plot_ros.create_semantic_bouding_box_with_label_marker(list_label, list_aabb, self.get_clock().now().to_msg())
            self.pub_object_marker.publish(marker)
            # Publish object cloud
            for i in range(len(keyframes)):
                for j in range(len(keyframes[i].object_clouds_global)):
                    pc = self.utility_ros.convert_open3d_to_ros_pointcloud2_msg(keyframes[i].object_clouds_global[j])
                    self.pub_keyframe_cloud.publish(pc)
            time.sleep(1.0)
    
def main(args=None):
    rclpy.init(args=args)
    node = ObjectMap()
    node.get_logger().info('Start object_map.')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
