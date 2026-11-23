from glob import glob

from setuptools import setup

package_name = 'television_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ARCoS-Lab',
    maintainer_email='aguterralgo@gmail.com',
    description='Stereo video to a WebXR headset (Open-TeleVision), head pose as a ROS topic.',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'television_stream = television_teleop.stream_node:main',
            'television_neck = television_teleop.neck_node:main',
        ],
    },
)
