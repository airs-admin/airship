import logging
import yaml

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class Config:
    def __init__(self, config_path):
        self.config_path = config_path
        with open(self.config_path, 'r') as file:
            logging.info(f'Read config file from: %s {self.config_path}')
            self.config = yaml.safe_load(file)

    def get(self, key, default_value=None):
        """Retrieves a configuration value for a specified key."""
        return self.config.get(key, default_value)