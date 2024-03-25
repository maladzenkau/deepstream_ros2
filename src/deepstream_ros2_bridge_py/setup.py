from setuptools import setup

package_name = 'deepstream_ros2_bridge_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dzmitry Maladzenkau',
    maintainer_email='dmolodenkov@gmail.com',
    description='Deepstream - ROS2 bridge with depth and color data synchronization',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kafka_publisher = deepstream_ros2_bridge_py.kafka_publisher:main',
            'latency_publisher = deepstream_ros2_bridge_py.latency_publisher:main',
            'validation_publisher = deepstream_ros2_bridge_py.validation:main',
        ],
    },
)
