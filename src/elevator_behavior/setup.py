from setuptools import find_packages, setup

package_name = 'elevator_behavior'

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
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'elevator_door_roi_detector_node = elevator_behavior.elevator_door_roi_detector:main',
            'elevator_spot_detector_node = elevator_behavior.elevator_spot_detector:main',
            'elevator_floor_detector_node = elevator_behavior.elevator_floor_detector:main'
            'elevator_in_and_out_node = elevator_behavior.elevator_in_and_out:main',

        ],
    },
)
