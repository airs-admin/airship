import logging
import time
import yaml

import numpy as np
from scipy.spatial.transform import Rotation as R
from pymycobot.mycobotsocket import MyCobotSocket

GRIPPER_CLOSE = 30
GRIPPER_RELEASE = 100
HOLD_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
INIT_POSE = [0.0, 0.0, 0.0, -65.0, 0.0, -45.0]
TOOL_REFERENCE = [0.0, 0.0, 100.0, 0.0, 0.0, 45.0]

class UtilityElephantRobot:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        # Read IP address and port from config
        self.robot_ip = self.config.get('robot', {}).get('ip')
        self.robot_port = self.config.get('robot', {}).get('port')
        self.arm_maximum_reach = self.config.get('arm_maximum_reach')
        self.min_angle_selcet = self.config.get('min_angle_select')
        self.max_angle_select = self.config.get('max_angle_select')
        # Load T_cam2real_gripper, T_depth2rgb, T_est2virtual_gripper
        self.cam2real_gripper_xyzrpy = self.config.get('cam2real_gripper_xyzrpy')
        self.est2virtual_gripper_xyzrpy = self.config.get('est2virtual_gripper_xyzrpy')
        # Initialize elephant robot
        self.my_cobot = MyCobotSocket(self.robot_ip, self.robot_port)
        self.initialize_cobot()
        
    def initialize_cobot(self):
        """Initialize MyCobotSocket and configure settings."""
        try:
            time.sleep(1)
            logging.info('MyCobotSocket connected!')
            self.my_cobot.set_tool_reference(TOOL_REFERENCE)
            time.sleep(1)
            self.my_cobot.set_end_type(1)
            time.sleep(2)
            self.my_cobot.send_angles(INIT_POSE, speed=10)
        except Exception as e:
            logging.info(f'Error initializing MyCobotSocket: {e}')
    
    def prepare_for_grasp(self):
        time.sleep(3)
        self.my_cobot.set_gripper_value(GRIPPER_RELEASE, speed=10, gripper_type=1)
        time.sleep(2)
        self.my_cobot.send_angles(INIT_POSE, speed=10)
        
    def get_arm_pose(self):
        arm_pose = []
        while not arm_pose:
            time.sleep(2)
            arm_pose = self.my_cobot.get_coords()
        arm_pose[:3] = [x / 1000.0 for x in arm_pose[:3]]
        return arm_pose
    
    def execute_grasp(self, grasp_coords):
        obj_distance = np.linalg.norm(grasp_coords[:3])
        if obj_distance < self.arm_maximum_reach:
            time.sleep(2)
            self.my_cobot.send_coords(grasp_coords, speed=1, mode=0)
            time.sleep(5)
            self.my_cobot.set_gripper_value(GRIPPER_CLOSE, speed=10, gripper_type=1)
            time.sleep(2)
            self.my_cobot.send_angles(HOLD_POSE, speed=10)
            time.sleep(5)
            return True
        return False
    
    def handle_place(self):
        time.sleep(1)
        # Replace the postion(option)
        self.my_cobot.send_angles(INIT_POSE, speed=10)
        time.sleep(4)
        self.my_cobot.set_gripper_value(GRIPPER_RELEASE, speed=30, gripper_type=1)
    
    def pose_to_transformation_matrix(self, pose):
        """Convert pose [x, y, z, rx, ry, rz] to a transformation matrix."""
        x, y, z, rx, ry, rz = pose
        rotation = R.from_euler('ZYX', [rz, ry, rx], degrees=True)
        r_matrix = rotation.as_matrix()
        T = np.eye(4)
        T[:3, :3] = r_matrix
        T[:3, 3] = [x, y, z]
        return T
    
    def matrix_to_euler_and_translation(self, T):
        """Convert transformation matrix to pose [x, y, z, rx, ry, rz]."""
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        r = R.from_matrix(rotation_matrix)
        euler_angles = r.as_euler('ZYX', degrees=True)
        x, y, z = translation
        rz, ry, rx = euler_angles
        return [x, y, z, rx, ry, rz]
    
    def select_best_grasp(self, grasp_pose, center_mask_point, trans_real2base):
        """Select the best grasp based on distance to the mask center and valid angles."""
        # Transformations
        trans_cam2real_gripper = self.pose_to_transformation_matrix(self.cam2real_gripper_xyzrpy)
        trans_est2virtual_gripper = self.pose_to_transformation_matrix(self.est2virtual_gripper_xyzrpy)
        distances = []
        # Compute distance from the mask center for each candidate grasp pose
        for i in range(len(grasp_pose.rotation_matrices)):
            trans_virtual2cam = np.eye(4)
            trans_virtual2cam[:3, :3] = grasp_pose.rotation_matrices[i]
            trans_virtual2cam[:3, 3] = grasp_pose.translations[i]
            grasp_coords = self.matrix_to_euler_and_translation(trans_virtual2cam)
            distance = np.sqrt(np.sum((grasp_coords[:3] - center_mask_point) ** 2))
            distances.append(distance)
        # Initialize variables
        min_distance_index = np.argmin(np.array(distances))
        attempt = 0
        found_valid_grasp = False
        # Try to find a valid grasp within the angular constraints
        while not found_valid_grasp and attempt < len(distances):
            center_mask_grasp = grasp_pose[min_distance_index:min_distance_index + 1]
            # Compute the transformation matrix from the virtual frame to the object grasp pose
            trans_virtual2cam = np.eye(4)
            trans_virtual2cam[:3, :3] = center_mask_grasp.rotation_matrices[0]
            trans_virtual2cam[:3, 3] = center_mask_grasp.translations[0]
            trans_est2base = trans_real2base @ trans_cam2real_gripper @ trans_virtual2cam @ trans_est2virtual_gripper
            # Get grasp coordinates and check if the angles are within the desired range
            grasp_coords = self.matrix_to_euler_and_translation(trans_est2base)
            center_mask_euler_angles = grasp_coords[3:]
            if self.min_angle_selcet <= center_mask_euler_angles[2] <= self.max_angle_select:
                found_valid_grasp = True
            else:
                # Mark this grasp as invalid and try the next best candidate
                distances[min_distance_index] = np.inf
                min_distance_index = np.argmin(np.array(distances))
                attempt += 1
        # Convert coordinates from meters to millimeters
        grasp_coords[:3] = [x * 1000.0 for x in grasp_coords[:3]]
        if not found_valid_grasp:
            logging.info('Transform extrapolation error: No valid grasp found within constraints.')
        return grasp_coords
