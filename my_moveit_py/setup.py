from setuptools import setup

package_name = 'my_moveit_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/moveit_py.yaml']),
        ('share/' + package_name + '/launch', ['launch/moveit_py_with_interbotix.launch.py']),
    ],
    install_requires=['setuptools', "fastmcp"],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='MoveItPy demo node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_demo = my_moveit_py.motion_planning_demo:main',
        ],
    },
)
