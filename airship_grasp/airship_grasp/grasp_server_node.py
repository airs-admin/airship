import threading

from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
import message_filters
import numpy as np
import rclpy

from airship_interface.srv import AirshipGrasp, SegmentationGrasping
from lib.utility_elephant_robot import UtilityElephantRobot
from lib.utility_graspnet import UtilityGraspNet

class GraspServer(Node):
    def __init__(self):
        super().__init__('grasp_server')
        # Declare and get config path
        self.declare_parameter('config', 'airship_grasp_config.yaml')
        self.config_path = self.get_parameter('config').get_parameter_value().string_value
        # Grasp Server
        self.grasp_srv = self.create_service(
            AirshipGrasp, '/airship_grasp/grasp_server', self.grasp_callback, callback_group=MutuallyExclusiveCallbackGroup()
        )
        # Segmentation client
        self.seg_cli = self.create_client(
            SegmentationGrasping, '/airship_perception/SegmentationGrasping', callback_group=MutuallyExclusiveCallbackGroup()
        )
        while not self.seg_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Seg service not available, waiting again...')
        self.get_logger().info('Initial Grasp_Server_node')  
        # Subscriptions
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.img_sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.img_sync.registerCallback(self.callback_img)
        # Other variables
        self.cv_bridge = CvBridge()
        # Initialize submodule
        self.utility_graspnet = UtilityGraspNet(self.config_path)
        self.utility_elephant_robot = UtilityElephantRobot(self.config_path)
        
    def callback_img(self, msg_color, msg_depth):
        self.color_image = msg_color
        self.depth_image = msg_depth
    
    def req_seg_sync(self, obj):
        request = SegmentationGrasping.Request()
        request.object = obj
        request.image = self.color_image
        result = self.seg_cli.call(request)
        mask = np.array(self.cv_bridge.imgmsg_to_cv2(result.mask, desired_encoding='mono8'), dtype=np.bool)
        rgb = np.array(self.cv_bridge.imgmsg_to_cv2(self.color_image, desired_encoding='bgr8'), dtype=np.float64) / 255.0
        depth = np.array(self.cv_bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='16UC1'))
        return rgb, depth, mask
    
    def request_segmentaion_service(self, obj):
        result = {}
        def target():
            result['data'] = self.req_seg_sync(obj)
        client_thread = threading.Thread(target=target)
        client_thread.start()
        client_thread.join()
        return result.get('data', ([], [], []))
    
    def _handle_pick_task(self, obj, response):
        # Call segmentation service
        self._logger.info("Start grasping ...")
        rgb, depth, mask = self.request_segmentaion_service(obj)
        self._logger.info("Thread Done ... Segmentation service completed")
        # Determine visibility
        if np.any(mask):
            # Preparing for grasp
            self.utility_elephant_robot.prepare_for_grasp()
            arm_pose = self.utility_elephant_robot.get_arm_pose()
            trans_gripper2base = self.utility_elephant_robot.pose_to_transformation_matrix(arm_pose)
            # calculate and select suitable grasp pose
            center_mask_point = self.utility_graspnet.get_3d_center_mask(mask, depth)
            grasp_pose = self.utility_graspnet.grasp_pose_estimation(rgb, depth, mask)
            grasp_coords = self.utility_elephant_robot.select_best_grasp(grasp_pose, center_mask_point, trans_gripper2base)
            # Determine Graspability
            if self.utility_elephant_robot.execute_grasp(grasp_coords):
                response.ret = AirshipGrasp.Response.SUCCESS
                self.get_logger().info(f'Successfully grasped {obj}')
            else:
                response.ret = AirshipGrasp.Response.FAILED_TO_REACH_OBJECT
                response.x, response.y, response.z = grasp_coords[:3]
                self.get_logger().info(f'Cannot reach {obj}')
        else:
            response.ret = AirshipGrasp.Response.FAILED_TO_FIND_OBJECT
            self.get_logger().info(f'Cannot find {obj}')
    
    def _handle_place_task(self, response):
        self.get_logger().info("Starting place task...")
        self.utility_elephant_robot.handle_place()
        response.ret = AirshipGrasp.Response.SUCCESS
        self.get_logger().info(f'Already delivered to the location')
        
    def grasp_callback(self, request, response):
        self.get_logger().info(f"Received request to {request.task} {request.obj}")
        if request.task == "pick":
            self._handle_pick_task(request.obj, response)
        elif request.task == "place":
            self._handle_place_task(response)
        return response

def main(args=None):
    rclpy.init(args=args)
    grasp = GraspServer()
    executor = MultiThreadedExecutor()
    executor.add_node(grasp)
    try:
        grasp.get_logger().info('Beginning Grasp, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        grasp.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
