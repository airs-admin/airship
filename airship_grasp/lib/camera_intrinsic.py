import yaml

class CameraIntrinsic():
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)   
        self.width = self.config.get('image_width')
        self.height = self.config.get('image_height')
        self.fx = self.config.get('fx')
        self.fy = self.config.get('fy')
        self.cx = self.config.get('cx')
        self.cy = self.config.get('cy')
        self.scale = self.config.get('depth_scale')