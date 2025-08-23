from setuptools import find_packages, setup

package_name = 'open_manipulator_x_qrreader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'open_manipulator_x_qrreader': ['sort.py'],   # ← 이 줄 추가!
    },
    include_package_data=True,                        # ← 이 줄 추가!
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',         # 필수
        'rclpy',              # ROS2 파이썬 클라이언트
        'opencv-python',      # OpenCV
        'ultralytics',        # YOLO
        'pyzbar',             # QR 코드 리더
        'numpy',              # 넘파이
        'serial',             # pyserial (UART)
    ],
    zip_safe=True,
    maintainer='seoyeon',
    maintainer_email='yunseoyeon59@gmail.com',
    description='ROS2 node for reading QR codes and publishing to control OpenManipulator-X',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qrreader_node = open_manipulator_x_qrreader.qrreader_node:main'
        ],
    },
)

