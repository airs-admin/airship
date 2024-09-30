import logging

import cv2
import numpy as np
import open3d as o3d

from lib.config import Config

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class ObjectDepth:
    def __init__(self, config_path):
        logging.info(f'Initilize ObjectDepth. Read configuration file from {config_path}.')
        # Depth camera intrinsic
        self.config = Config(config_path)
        self.K = np.array(self.config.get('K'))
        self.image_width = self.config.get('image_width')
        self.image_height = self.config.get('image_height')
        self.cx = self.K[0][2]
        self.cy = self.K[1][2]
        self.fx = self.K[0][0]
        self.fy = self.K[1][1]
        # The extrinsic parameters from camera_optical_frame to base_link
        self.R = np.array(self.config.get('R'))
        self.T = np.array(self.config.get('T')).reshape((3, 1))
        self.transformation = np.eye(4)
        self.transformation[:3, :3], self.transformation[:3, 3] = self.R, self.T.flatten()
        # Erosion algorithm parameters for filtering out edge errors in object segmentation mask
        self.erosion_size_kernel = self.config.get('erosion_size_kernel', 6)
        self.erosion_num_iterations = self.config.get('erosion_num_iterations', 2)
        self.erosion_kernel = np.ones((self.erosion_size_kernel, self.erosion_size_kernel), np.uint8)

    def covert_to_base_link(self, cloud, transform=None):
        if transform is None:
            transform = self.transformation
        cloud.transform(self.transformation)
        return cloud
        
    def erod_masks(self, masks):
        # Erode all masks
        eroded_masks = np.array([cv2.erode(mask, self.erosion_kernel, iterations=self.erosion_num_iterations) for mask in masks])
        # Check non-zero masks
        valid_indices = []
        for i, eroded_mask in enumerate(eroded_masks):
            if np.any(eroded_mask):
                valid_indices.append(i)
            else:
                logging.info(f'Remove index {i} after erosion.')
        valid_eroded_masks = eroded_masks[valid_indices]
        return valid_eroded_masks, valid_indices

    def get_clouds_by_masks(self, masks, depth, image, do_mask_erosion=True, to_base_link=True):
        # Do mask erosion
        if do_mask_erosion:
            masks, indices = self.erod_masks(masks)
        else:
            indices = list(range(len(masks)))
        clouds = []
        updated_indices = []
        for ind_mask, mask in enumerate(masks):
            pt_object = np.where(mask > 0)
            u_coords, v_coords = pt_object[1], pt_object[0]
            depth_values = np.array([self.get_depth_from_mat(depth, u, v) for u, v in zip(u_coords, v_coords)])
            # Filter out invalid depth values
            valid_mask = (depth_values >= 0) & ~np.isinf(depth_values)
            valid_depths = depth_values[valid_mask]
            u_coords, v_coords = u_coords[valid_mask], v_coords[valid_mask]
            if len(valid_depths) > 0:
                points = np.array([self.get_point_from_depth(u, v, depth) for u, v, depth in zip(u_coords, v_coords, valid_depths)])
                colors = image[v_coords, u_coords, :3] / 255.0
                cloud = self.get_colored_cloud(points, colors)
                if to_base_link: 
                    cloud = self.covert_to_base_link(cloud)
                updated_indices.append(indices[ind_mask])
                clouds.append(cloud)
            else:
                logging.info(f'Remove index {indices[ind_mask]} after valid depth selection.')
        return clouds, np.array(updated_indices).astype(int)

    def get_colored_cloud(self, points, colors):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        return cloud

    def get_depth_from_mat(self, depth_map, u, v):
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            idx = u + self.image_width * v
            depth_value = depth_map[idx]
        else:
            depth_value = None
            logging.info(f'Index ({u}, {v}) is out of image.')
        return depth_value

    def get_point_from_depth(self, u, v, depth_value):
        x = (u - self.cx) * depth_value / self.fx
        y = (v - self.cy) * depth_value / self.fy
        z = depth_value
        return x, y, z

