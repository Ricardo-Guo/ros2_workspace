import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'your_robot_tasks'

# 这里指向 setup.py 所在目录
here = os.path.dirname(os.path.abspath(__file__))

# 获取 models/box 目录下的所有文件（相对路径）
box_files = glob(os.path.join('models', 'box', '*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 必需的资源索引
        ('share/ament_index/resource_index/packages',
         ['resource/your_robot_tasks']),
        # package.xml
        ('share/your_robot_tasks', ['package.xml']),
        # 安装 models/box 下的所有文件
        ('share/your_robot_tasks/models/box', box_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardo',
    maintainer_email='12313411@mail.sustech.edu.cn',
    description='Pick and Place tasks for your robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place_node = your_robot_tasks.pick_place_node:main',
            
        ],
    },
)
