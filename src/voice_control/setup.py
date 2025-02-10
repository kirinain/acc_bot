from setuptools import setup

package_name = 'voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiti',
    maintainer_email='kiti@example.com',
    description='Voice Control Package for ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'st = voice_control.st:main',
            'robot_controller = voice_control.robot_controller:main', 
        ],
    },
)
