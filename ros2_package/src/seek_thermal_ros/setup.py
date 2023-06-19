import os
from glob import glob
from setuptools import setup

package_name = 'seek_thermal_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer Tuteja',
    maintainer_email='sameer.tuteja05@gmail.com',
    description='Seek Thermal Camera Ros2 Wrapper',
    license='CDDL-1.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_publisher = seek_thermal_ros.thermal_publisher:main',            
        ],
    },
)
