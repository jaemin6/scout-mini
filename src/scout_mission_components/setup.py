from setuptools import setup

package_name = 'scout_mission_components'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission_components_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddy',
    maintainer_email='eddy@example.com',
    description='Mission components for Scout robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector = scout_mission_components.qr_detector:main',
            'robot_rotator_node = scout_mission_components.robot_rotator_node:main',
            'amcl_reset_node = scout_mission_components.amcl_reset_node:main',
            'nav2_commander = scout_mission_components.nav2_commander:main',
        ],
    },
)

