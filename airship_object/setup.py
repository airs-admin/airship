from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'airship_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'lib'), glob('lib/*.py')),
        (os.path.join('share', package_name, 'lib'), glob('lib/*.so')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airsbot2',
    maintainer_email='airship@cuhk.edu.cn',
    description='airship_object',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_map_node = airship_object.object_map_node:main'
        ],
    },
)
