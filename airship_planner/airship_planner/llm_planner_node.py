import os
import time
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import rclpy
import yaml

from airship_interface.srv import AirshipInstruct, AirshipGrasp, AirshipNav
from airship_planner.task_planner import LLAMA_Task_Planner

class LLMPlanner(Node):

    def __init__(self):
        super().__init__('llm_planner')

        self._busy = False

        curr_file_dir = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('config', 'config/config.yaml')
        self.config_path = self.get_parameter('config').get_parameter_value().string_value
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.declare_parameter('semantic_map_dir', os.path.join(curr_file_dir, '../map/'))
        self.declare_parameter('semantic_map_file', 'semantic_map.yaml')
        self.declare_parameter('llm_server_url', 'localhost')

        semantic_map = self.get_parameter('semantic_map_dir').value + self.config.get('semantic_map_file')
        server_url = self.config.get('llm_server_url')
        self._task_planner = LLAMA_Task_Planner(server_url, semantic_map)

        service_cb_group = MutuallyExclusiveCallbackGroup()
        self._planner_srv = self.create_service(
            AirshipInstruct,
            '/airship_planner/planner_server',
            self.instruct_callback,
            callback_group=service_cb_group)

        client_cb_group = MutuallyExclusiveCallbackGroup()
        self._grasp_cli = self.create_client(AirshipGrasp,
                                             '/airship_grasp/grasp_server',
                                             callback_group=client_cb_group)
        self._nav_cli = self.create_client(
            AirshipNav,
            '/airship_navigation/navigate_to_pose',
            callback_group=client_cb_group)

        while not self._grasp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Grasping service not available, waiting again...')
        self._grasp_req = AirshipGrasp.Request()

        while not self._nav_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Navigation service not available, waiting again...')
        self._nav_req = AirshipNav.Request()

    def _grasp_request_sync(self, task, obj):
        self._grasp_req.task = task
        self._grasp_req.obj = obj
        self._grasp_ret = self._grasp_cli.call(self._grasp_req)

    def _nav_request_sync(self, x, y, theta):
        self._nav_req.x = x
        self._nav_req.y = y
        self._nav_req.theta = theta
        self._nav_ret = self._nav_cli.call(self._nav_req)

    def _task_scheduler(self, task_list):
        self.get_logger().info("Start LLM planner scheduling...")

        for job in task_list:
            task = job[0]
            action = job[1]
            if task == 'go_to':
                self.get_logger().info("Start navigating to %s at [%f, %f]..." %
                                       (action[0], action[1][0], action[1][1]))

                client_thread = threading.Thread(target=self._nav_request_sync,
                                                 args=(action[1][0],
                                                       action[1][1],
                                                       action[1][2]))
                client_thread.start()
                client_thread.join()
                res = self._nav_ret
                if res.status != AirshipNav.Response.SUCCESS:
                    self.get_logger().warn(
                        "Cannot navigate to %s. Navigating error: %s" %
                        (action[0], res.status))
                    return AirshipInstruct.Response.NAV_FAIL
                else:
                    self.get_logger().info("Reach %s..." % action[0])

            elif task == 'pick_up':
                self.get_logger().info("Start Picking %s..." % action[0])
                client_thread = threading.Thread(
                    target=self._grasp_request_sync, args=("pick", action[0]))
                client_thread.start()
                client_thread.join()
                res = self._grasp_ret
                if res.ret == AirshipGrasp.Response.SUCCESS:
                    self.get_logger().info("End Picking %s..." % action[0])
                else:
                    if res.ret == AirshipGrasp.Response.FAILED_TO_FIND_OBJECT:
                        self.get_logger().warn(
                            "Cannot find object %s at [%f, %f]. Grasping error: %s "
                            % (action[0], res.x, res.y, res.ret))
                    elif res.ret == AirshipGrasp.Response.FAILED_TO_REACH_OBJECT:
                        self.get_logger().warn(
                            "Cannot reach object %s at [%f, %f]. Grasping error: %s "
                            % (action[0], res.x, res.y, res.ret))
                    else:
                        self.get_logger().warn(
                            "Unknown grasping error, return: %d " % (res.ret))
                    return AirshipInstruct.Response.PICK_FAIL

            elif task == 'place':
                self.get_logger().info("Start Placing %s..." % action[0])
                client_thread = threading.Thread(
                    target=self._grasp_request_sync, args=("place", action[0]))
                client_thread.start()
                client_thread.join()
                res = self._grasp_ret
                if res.ret != AirshipGrasp.Response.SUCCESS:
                    self.get_logger().warn("Placing %s error: %s" %
                                           (action[0], res.status))
                    return AirshipInstruct.Response.PLACE_FAIL
                else:
                    self.get_logger().info("Finish placing %s..." % action[0])
            else:
                self.get_logger().warn("I cannot handle the %s task..." % task)
                return AirshipInstruct.Response.UNKOWN_TASK

        self.get_logger().info("LLM planning done...")
        return AirshipInstruct.Response.SUCCESS

    def instruct_callback(self, request, response):
        if self._busy:
            self.get_logger().warning('The robot is working.')
            response.ret = AirshipInstruct.Response.ROBOT_BUSY
            return response

        instruct_msg = request.msg
        if not len(instruct_msg):
            self.get_logger().warning('Receive an empty instruction')
            response.ret = AirshipInstruct.Response.EMPTY_INST
            return response

        self._busy = True
        self.get_logger().info(
            'Parsing users\' instruction: {}'.format(instruct_msg))
        task_list = self._task_planner.get_tasks(instruct_msg)
        self.get_logger().info("LLM planned task list: {}".format(task_list))
        response.ret = self._task_scheduler(task_list)

        self._busy = False
        return response

def main(args=None):
    rclpy.init(args=args)
    planner = LLMPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)
    try:
        planner.get_logger().info(
            'Beginning LLM_Planner, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        planner.get_logger().info('Keyboard interrupt, shutting down.\n')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
