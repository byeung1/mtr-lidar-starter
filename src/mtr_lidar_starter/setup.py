from setuptools import setup
from glob import glob
setup(name='mtr_lidar_starter', version='0.1.0',
      packages=['mtr_lidar_starter'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/mtr_lidar_starter']),
                  ('share/mtr_lidar_starter', ['package.xml']),
                  ('share/mtr_lidar_starter/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='MTR team', maintainer_email='maintainer@example.com',
      description='Hardware-independent LiDAR starter', license='MIT',
      entry_points={'console_scripts': [
          'lidar_node = mtr_lidar_starter.lidar_node:main',
          'costmap_viewer = mtr_lidar_starter.costmap_viewer:main']})
