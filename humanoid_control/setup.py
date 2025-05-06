from setuptools import find_packages, setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grail',
    maintainer_email='szheng2@g.clemson.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = humanoid_control.motor_control:main',
            'slider_gui = humanoid_control.slider_gui:main',
            'slider_gui_hand = humanoid_control.slider_gui_hand:main',
            'trajectory_to_jointstate_bridge = humanoid_control.trajectory_to_jointstate_bridge:main',
            'initial_pose_publisher = humanoid_control.initial_pose_publisher:main',
            'joint_state_rate_adapter = humanoid_control.joint_state_rate_adapter:main',
        ],
    },
)
