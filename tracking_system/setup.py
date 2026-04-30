from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tracking_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # dashboard templates
        (os.path.join('share', package_name, 'templates'),
            glob('tracking_system/templates/*.html')),

        # dashboard static files
        (os.path.join('share', package_name, 'static', 'css'),
            glob('tracking_system/static/css/*.css')),
        (os.path.join('share', package_name, 'static', 'js'),
            glob('tracking_system/static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TF based leader follower tracking system',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tf_node = tracking_system.aruco_tf_node:main',
            'follow_target_tf_node = tracking_system.follow_target_tf_node:main',
            'follower_controller_node = tracking_system.follower_controller_node:main',
            'robot_driver_node = tracking_system.robot_driver_node:main',
            'keyboard_node = tracking_system.keyboard_node:main',
            'dashboard_node = tracking_system.dashboard_node:main',
        ],
    },
)