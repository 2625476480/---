from setuptools import find_packages, setup

package_name = 'e10_setgoal'

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
    maintainer='xiaom',
    maintainer_email='790875685@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'nav_to_pose = e10_setgoal.nav_to_pose:main',
        'waypoint_follower = e10_setgoal.waypoint_follower:main',
        'init_robot_pose = e10_setgoal.init_robot_pose:main',
        'init_and_setGoal = e10_setgoal.init_and_setGoal:main',
        'init_and_setGoal_2 = e10_setgoal.init_and_setGoal_2:main',
        'point_detector_node = e10_setgoal.point_detector_node:main' #记得加逗号
        ],
    },
)
