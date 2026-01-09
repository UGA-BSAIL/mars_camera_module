from setuptools import find_packages, setup

package_name = 'ros_hw_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ros_hw_monitor_msgs'],
    zip_safe=True,
    maintainer='Daniel Petti',
    maintainer_email='daniel.petti@ufl.edu',
    description='Package for profiling and hardware monitoring on ROS devices.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hw_monitor = ros_hw_monitor.hw_monitor_node:main'
        ],
    },
)
