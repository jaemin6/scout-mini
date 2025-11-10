from setuptools import find_packages, setup

package_name = 'my_nav2_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ⚠️ (수정) ROS 2의 표준 형식으로 resource 파일을 등록합니다.
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='eddy',
    maintainer_email='gumt555@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # extras_require는 Nav2 제어에 필수적이지 않아 제거하거나 최소화합니다.
    entry_points={
        'console_scripts': [
            'go_forward = my_nav2_controller.go_forward:main',
        ],
    },
)

