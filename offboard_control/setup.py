from glob import glob
from setuptools import setup

setup(name='offboard_control',version='0.1.0',
      packages=['offboard_control','mppi_series'],
      package_dir={'mppi_series':'../simulation/mppi_series'},
      data_files=[('share/ament_index/resource_index/packages',['resource/offboard_control']),
                  ('share/offboard_control',['package.xml']),
                  ('share/offboard_control/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','numpy'],zip_safe=False,
      maintainer='AFM',maintainer_email='developer@example.com',license='Apache-2.0',
      description='MAVROS normalized thrust and body-rate MPPI interface',
      entry_points={'console_scripts':['mppi_interface = offboard_control.mppi_node:main',
                                      'mppi_flight = offboard_control.mppi_flight_node:main']})
