from setuptools import find_packages, setup

package_name = 'path_and_motion_planning'

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
    maintainer='Vedika Chauhan',
    maintainer_email='vedika.chauhan@outlook.com',
    description='Package that enables the robot to navigate towards global goals while avoiding obstacles',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
