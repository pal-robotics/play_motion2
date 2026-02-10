from setuptools import find_packages
from setuptools import setup

package_name = 'play_motion2_cli'

setup(
    name=package_name,
    version='1.8.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Isaac Acevedo',
    author_email='isaac.acevedo@pal-robotics.com',
    maintainer='Isaac Acevedo',
    maintainer_email='isaac.acevedo@pal-robotics.com',
    keywords=[],
    description='The play_motion command for ROS 2 command line tools.',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'play_motion = play_motion2_cli.command.play_motion:PlayMotionCommand',
        ],
        'ros2cli.extension_point': [
            'play_motion2_cli.verb = play_motion2_cli.verb:VerbExtension',
        ],
        'play_motion2_cli.verb': [
            'list = play_motion2_cli.verb.list:ListVerb',
            'info = play_motion2_cli.verb.info:InfoVerb',
            'run = play_motion2_cli.verb.run:RunVerb',
        ],
    }
)
