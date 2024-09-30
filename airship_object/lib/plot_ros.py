import logging

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class PlotROS:
    def __init__(self):
        """Initialize the class."""
        self.color_map = self.generate_color_map(255)

    def create_bounding_box_marker(self, vertices, marker_id, timestamp, frame_id='map'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = "bounding_box"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        edges = [
            (0, 1), (1, 7), (7, 2), (2, 0),
            (3, 6), (6, 4), (4, 5), (5, 3),
            (0, 3), (1, 6), (7, 4), (2, 5)
        ]
        for edge in edges:
            p1 = vertices[edge[0]]
            p2 = vertices[edge[1]]
            marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
            marker.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))
        return marker
    
    def create_object_nav_goal_marker(self, list_object_nav_goal, timestamp, frame_id='map'):
        marker_array = MarkerArray()
        for i, (x, y, heading) in enumerate(list_object_nav_goal):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "arrows"
            marker.id = i * 2
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            quat_z = np.sin(heading / 2.0)
            quat_w = np.cos(heading / 2.0)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = quat_z
            marker.pose.orientation.w = quat_w
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            # Text
            text_marker = self.create_text_marker((x, y, 0), str(i), i * 2 + 1, timestamp)
            marker_array.markers.append(text_marker)
        return marker_array

    def create_semantic_bouding_box_with_label_marker(self, list_label, list_vertices, timestamp, frame_id='map'):
        marker_array = MarkerArray()
        marker_id = 0
        for index, (label, vertices) in enumerate(zip(list_label, list_vertices)):
            box_marker = self.create_bounding_box_marker(vertices, marker_id, timestamp, frame_id=frame_id)
            marker_array.markers.append(box_marker)
            marker_id += 1
            highest_point = vertices[np.argmax(vertices[:, 2])]
            combined_string = '{}: {}'.format(index, label)
            text_marker = self.create_text_marker(highest_point, combined_string, marker_id, timestamp, frame_id=frame_id)
            marker_array.markers.append(text_marker)
            marker_id += 1
        return marker_array

    def create_text_marker(self, position, text, marker_id, timestamp, frame_id='map'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = "label"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.2
        marker.text = text
        return marker
    
    def generate_color_map(self, num_classes):
        np.random.seed(57)
        color_map = {}
        for i in range(num_classes):
            color_map[i] = tuple(np.random.randint(0, 256, size=3).tolist())
        return color_map