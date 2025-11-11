from setuptools import find_packages, setup

package_name = 'scout_mission_components'

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
    maintainer='eddy',
    maintainer_email='gumt555@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 기존 독립 실행 스크립트가 없으므로 비워둡니다.
        ],
        # ----------------------------------------------------
        # 🌟🌟 파이썬 컴포넌트 등록 섹션 (필수) 🌟🌟
        # ----------------------------------------------------
        'rclpy_components': [
            'amcl_reset = scout_mission_components.amcl_reset:create_node',
            'nav2_commander = scout_mission_components.nav2_commander1:create_node',
            'qr_detector = scout_mission_components.qr_detector:create_node',
            'robot_rotator = scout_mission_components.robot_rotator:create_node',
        ],
    },
)
