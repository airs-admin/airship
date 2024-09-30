import logging

from cv_bridge import CvBridge
from rclpy.clock import Clock
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class UtilityROS:
    def __init__(self):
        """Initialize the class."""
        self.cv_bridge = CvBridge()

    def convert_open3d_to_ros_pointcloud2_msg(self, cloud, stamp=None, frame_id='map'):
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud_data = [
            (*point, (int(color[0] * 255) << 16) | (int(color[1] * 255) << 8) | int(color[2] * 255))
            for point, color in zip(points, colors)
        ]
        
        header = Header(stamp=stamp if stamp is not None else Clock().now().to_msg(), frame_id=frame_id)
        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        return cloud_msg

    def depth_message_to_mat(self, depth_map_msg):
        return np.frombuffer(depth_map_msg.data, dtype=np.float32).tolist()

    def get_time_from_header(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9
    
    def image_message_to_mat(self, img_msg, decode=cv2.COLOR_BGRA2RGB):
        return cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(img_msg), decode)
    
    def odometry_message_to_position_and_orientation(self, odom_msg):
        position = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]
        orintation = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        return position, orintation
    
    def pose_stamped_message_to_position_and_orientation(self, pose_stamped_msg):
        position = [pose_stamped_msg.pose.position.x, pose_stamped_msg.pose.position.y, pose_stamped_msg.pose.position.z]
        orintation = [pose_stamped_msg.pose.orientation.x, pose_stamped_msg.pose.orientation.y, pose_stamped_msg.pose.orientation.z, pose_stamped_msg.pose.orientation.w]
        return position, orintation