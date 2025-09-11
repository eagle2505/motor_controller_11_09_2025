from setuptools import find_packages, setup
import glob
import os

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/forward_mover.launch.py']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml') + glob.glob('config/*.rviz')),
        ('share/' + package_name + '/maps', glob.glob('maps/*.yaml') + glob.glob('maps/*.pgm')),
        ('share/' + package_name + '/description', ['description/robot.urdf.xacro']),
        ('share/' + package_name + '/behavior_trees', glob.glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roamio42',
    maintainer_email='roamio42@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
            entry_points={
            'console_scripts': [
                 'keyboard_control = motor_control.keyboard_control:main',
                 'motor_controller = motor_control.motor_controller:main',
                 'odometry_publisher = motor_control.odometry_publisher:main',
                 'cmd_vel_to_motors = motor_control.cmd_vel_to_motors:main',
                 'forward_mover = motor_control.forward_mover:main',
                 'scan_frame_remapper = motor_control.scan_frame_remapper:main',
            ],
        },
)
