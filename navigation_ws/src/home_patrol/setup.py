from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'home_patrol'


def package_files(directory):
    data_files = []

    for path, directories, filenames in os.walk(directory):
        files = [
            os.path.join(path, filename)
            for filename in filenames
        ]

        if files:
            install_path = os.path.join(
                'share',
                package_name,
                path
            )

            data_files.append(
                (install_path, files)
            )

    return data_files


data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),

    # launch 파일 설치
    (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')
    ),
]

# Custom Gazebo model 설치
data_files += package_files('models')


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=data_files,

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='robotics',
    maintainer_email='el06155@naver.com',

    description='ROS2 Home Patrol & Person Alert System',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'waypoint_patrol = home_patrol.waypoint_patrol:main',
        ],
    },
)