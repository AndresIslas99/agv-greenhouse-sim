from setuptools import setup
from glob import glob

package_name = 'agv_sim_validation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andres Islas',
    maintainer_email='andresislas2107@gmail.com',
    description='AI validation overlay for the AGV Isaac Sim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ground_truth_publisher = agv_sim_validation.ground_truth_publisher:main',
            'localization_monitor   = agv_sim_validation.localization_monitor:main',
            'events_detector        = agv_sim_validation.events_detector:main',
            'episode_tracker        = agv_sim_validation.episode_tracker:main',
            'sim_api                = agv_sim_validation.sim_api:main',
        ],
    },
)
