from setuptools import setup

package_name = 'path_follow_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/parameters_dir', ['data_files/parameters.json'])
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='masum',
    maintainer_email='your_email@example.com',
    description='Combined path planner and executor for TurtleBot in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_and_follow = path_follow_executor.plan_and_follow:main',
        ],
    },
)

