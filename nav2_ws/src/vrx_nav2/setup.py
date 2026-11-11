from glob import glob
from pathlib import Path
from setuptools import setup

setup(name='vrx_nav2', version='0.1.0', packages=['vrx_nav2'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/vrx_nav2']),
                  ('share/vrx_nav2', ['package.xml']),
                  *[(f'share/vrx_nav2/{d}',
                     [path for path in glob(f'{d}/*') if Path(path).is_file()])
                    for d in ('launch', 'config', 'behavior_trees')]],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'ground_truth = vrx_nav2.ground_truth:main',
          'cloud_filter = vrx_nav2.cloud_filter:main',
          'thrust_adapter = vrx_nav2.thrust_adapter:main',
          'waypoints = vrx_nav2.waypoints:main',
          'click_goal = vrx_nav2.click_goal:main']})
