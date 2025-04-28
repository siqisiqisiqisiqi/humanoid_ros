from setuptools import find_packages, setup

package_name = 'servo_keyboard_py'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_servo = servo_keyboard_py.keyboard_servo:main',
            'humanoid_keyboard_servo = servo_keyboard_py.keyboard_servo_humanoid:main',
            'static_joint_publisher = servo_keyboard_py.static_joint_publisher:main',
        ],
    },
)
