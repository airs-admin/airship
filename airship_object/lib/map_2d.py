import logging

import numpy as np

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class Map2D:
    def __init__(self):
        pass

    def find_nav_pose_based_on_observe_pose(self, list_pose_2d, objects):
        list_pose_2d_w_heading = np.zeros((len(list_pose_2d), 3))
        list_object_label = [None] * len(list_pose_2d)
        for i, pose_2d in enumerate(list_pose_2d):
            object_positions = np.array(objects[i].position)
            object_positions_2d = object_positions[:, :2]
            distances = np.linalg.norm(object_positions_2d - pose_2d, axis=1)
            min_index = np.argmin(distances)
            best_object_positions_2d = object_positions_2d[min_index]
            best_object_heading = np.arctan2(pose_2d[1] - best_object_positions_2d[1], pose_2d[0] - best_object_positions_2d[0])
            list_pose_2d_w_heading[i, 0] = best_object_positions_2d[0]
            list_pose_2d_w_heading[i, 1] = best_object_positions_2d[1]
            list_pose_2d_w_heading[i, 2] = best_object_heading
            list_object_label[i] = objects[i].object_labels
        return list_pose_2d_w_heading.tolist(), list_object_label