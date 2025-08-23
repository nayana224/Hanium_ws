from setuptools import find_packages, setup

package_name = 'open_manipulator_x_uart'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uart_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_receiver = open_manipulator_x_uart.uart_node:main',
        ],
    },
)
