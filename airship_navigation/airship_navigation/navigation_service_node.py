import math

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
import rclpy
import tf_transformations
import tf2_ros

from airship_interface.srv import AirshipNav

class NavigationService(Node):
    def __init__(self, node_name='navigation_service'):
        super().__init__(node_name=node_name)
        # Declare parameters
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('time_scaling_factor', 5.0)
        self.declare_parameter('min_runtime', 20.0)
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.time_scaling_factor = self.get_parameter('time_scaling_factor').get_parameter_value().double_value
        self.min_runtime = self.get_parameter('min_runtime').get_parameter_value().double_value

        # Initilize service and simple navigator commander.
        self.srv = self.create_service(AirshipNav, '/airship_navigation/navigate_to_pose', self.navigate_to_pose_callback)
        self.navigator = BasicNavigator(node_name='navigator_node')

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def calculate_path_length(self, path):
        length = 0.0
        poses = path.poses
        if len(poses) < 2:
            return length

        for i in range(1, len(poses)):
            # Get the coordinates of two adjacent Poses.
            pos1 = poses[i - 1].pose.position
            pos2 = poses[i].pose.position

            # Calculate the Euclidean distance.
            distance = math.sqrt((pos2.x - pos1.x) ** 2 +
                                 (pos2.y - pos1.y) ** 2 +
                                 (pos2.z - pos1.z) ** 2)

            # Accumulate the distance.
            length += distance

        return length
    
    def navigate_to_pose_callback(self, request, response):
        self.get_logger().info('Received navigation request to x: {}, y: {}, theta: {}'.format(request.x, request.y, request.theta))

        # Update current_pose to the latest value.
        current_pose = self.update_current_pose_from_tf()
        if current_pose is None:
            self.get_logger().info('Failed to get current pose.')
            response.status = AirshipNav.Response.FAILED
            return response
        
        # Set target pose.
        goal_pose = self.set_posestamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = request.x
        goal_pose.pose.position.y = request.y
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, request.theta)
        goal_pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Get path.
        path = self.navigator.getPath(current_pose, goal_pose)

        # Empty path means target inside obstacle.
        if not path:
            self.get_logger().info('FAILED_TO_FIND_PATH')
            response.status = AirshipNav.Response.FAILED_TO_FIND_PATH
            return response

        # If target position is inside obstacle avoidance distance, planner will give out a cloest point.
        final_pose = path.poses[-1].pose
        goal_changed = not (final_pose.position.x == request.x and final_pose.position.y == request.y)

        # Start navigation.
        # Estimate time needed.
        distance = self.calculate_path_length(path)
        timeout_threshold = max(distance / self.max_speed * self.time_scaling_factor, self.min_runtime)
        timeout_occurred = False
        self.navigator.goToPose(goal_pose)
        # Status monitor & time out check.
        while not self.navigator.isTaskComplete():
          feedback = self.navigator.getFeedback()
          # self.get_logger().info(f'Navigation feedback: {feedback}')
          # Timeout threashold
          if feedback.navigation_time.sec > timeout_threshold:
            self.get_logger().info(f'Timeout: navigation time > {timeout_threshold}')
            timeout_occurred = True
            self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if goal_changed:
                self.get_logger().info('SUCCES_BY_REACHING_CLOEST_POINT')
                response.status = AirshipNav.Response.SUCCES_BY_REACHING_CLOEST_POINT
            else:
                self.get_logger().info('SUCCESS')
                response.status = AirshipNav.Response.SUCCESS
        else:
            if timeout_occurred:
                self.get_logger().info('FAILED_TIMEOUT')
                response.status = AirshipNav.Response.FAILED_TIMEOUT
            else:
                self.get_logger().info('FAILED')
                response.status = AirshipNav.Response.FAILED
        return response
    
    def set_posestamped(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        return pose

    def update_current_pose_from_tf(self):
        try:
            # Lookup the latest transform between 'map' and 'base_link'
            timeout = rclpy.duration.Duration(seconds=0.15)
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout)

            # Create a new PoseStamped object and populate it with the transform data
            current_pose = self.set_posestamped()
            current_pose.header.stamp = transform.header.stamp
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation
            return current_pose

        except tf2_ros.LookupException:
            self.get_logger().warn('Transform from map to base_link not found')
        except tf2_ros.ConnectivityException:
            self.get_logger().warn('Transform connectivity error')
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('Transform extrapolation error')
        return None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
