import logging
import yaml

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class IONavGoal:
    def __init__(self, label="", x=0.0, y=0.0, theta=0.0):
        self.label = label
        self.x = x
        self.y = y
        self.theta = theta

    def to_dict(self):
        return {'label': str(self.label), 'x': float(self.x), 'y': float(self.y), 'theta': float(self.theta)}

    @classmethod
    def create_objects(cls, list_label, list_pose):
        if len(list_label) != len(list_pose):
            raise ValueError("The length of list_label and list_pose must be the same.")
        return [cls(label, pose[0], pose[1], pose[2]) for label, pose in zip(list_label, list_pose)]

    @staticmethod
    def find_objects_by_label(data, label):
        return [{key: value} for key, value in data.items() if value['label'] == label]

    @staticmethod
    def load_yaml_to_dict(filename):
        try:
            with open(filename, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            logging.info(f'Error: File {filename} not found.')
        except yaml.YAMLError as exc:
            logging.info(f'Error parsing YAML file: {exc}')
        return {}

    @staticmethod
    def write_nav_goal_to_yaml(objects, filename):
        data = {f"object{index + 1}": obj.to_dict() for index, obj in enumerate(objects)}
        try:
            with open(filename, 'w') as file:
                yaml.dump(data, file)
        except IOError as e:
            logging.info(f'Error writing to file {filename}: {e}')