from glob import glob

from setuptools import setup

package_name = 'gello_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ARCoS-Lab',
    maintainer_email='aguterralgo@gmail.com',
    description='GELLO teleoperation for the ARCoS-Lab Franka Panda.',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            'gello_driver = gello_teleop.gello_driver:main',
            'gello_calibrator = gello_teleop.gello_calibrator:main',
            'gello_get_offset = gello_teleop.gello_get_offset:main',
        ],
    },
)
