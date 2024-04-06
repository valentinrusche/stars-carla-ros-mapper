from setuptools import setup

package_name = 'carla_stars_ros_mapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        package_name + "/nodes",
        package_name + "/nodes/carla_implementations"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Rusche',
    maintainer_email='valentin.rusche@udo.edu',
    description='Bridge to read Carla ROS topics and write them to STARS compatible topics',
    license='AGPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_stars_ros_mapper = carla_stars_ros_mapper.main:main',
        ],
    },
)
