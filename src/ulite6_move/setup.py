from setuptools import find_packages, setup

package_name = 'ulite6_move'

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
    maintainer='ricard',
    maintainer_email='ric4rd11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_move = ulite6_move.square_move:main',
            'move_linear = ulite6_move.move_linear:main',
            'xTrack = ulite6_move.xTrack:main',
            'xTrack_servo = ulite6_move.xTrack_servo:main',
            'pose_publisher = ulite6_move.pose_publisher:main',
            'xyTrack_servo = ulite6_move.xyTrack_servo:main',
            'xyzTrack_servo = ulite6_move.xyzTrack_servo:main',
            'xrTrack_servo = ulite6_move.xrTrack_servo:main',
            'xyrTrack_servo = ulite6_move.xyrTrack_servo:main',
            'xyzrTrack_servo = ulite6_move.xyzrTrack_servo:main',
            'predict = ulite6_move.predict:main',
            'videoStream = ulite6_move.videoStream:main',
            'data_logger = ulite6_move.data_logger:main',
        ]
    },
)
