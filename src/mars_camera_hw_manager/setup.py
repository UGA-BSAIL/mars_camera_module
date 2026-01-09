from setuptools import find_packages, setup

package_name = 'mars_camera_hw_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Petti',
    maintainer_email='daniel.petti@ufl.edu',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fan_controller = mars_camera_hw_manager.fan_controller_node:main',
            'hardware_info = mars_camera_hw_manager.hardware_info_node:main',
            'manager = mars_camera_hw_manager.manager_node:main',
            'shutdown = mars_camera_hw_manager.shutdown_node:main',
        ],
    },
)
