from setuptools import setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Control nodes for my_robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_follower = my_robot_control.camera_follower:main',
            'dynamic_obstacles = my_robot_control.dynamic_obstacles:main',
        ],
    },
)

