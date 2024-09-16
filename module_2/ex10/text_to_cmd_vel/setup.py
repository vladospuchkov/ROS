from setuptools import setup

package_name = 'text_to_cmd_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Text to cmd_vel publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_cmd_vel_node = text_to_cmd_vel.text_to_cmd_vel_node:main'
        ],
    },
)

