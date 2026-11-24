from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xgo_lite2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='user@example.com',
    description='ROS 2 driver node wrapping xgolib.py for XGO-Lite2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xgo_node = xgo_lite2_driver.xgo_node:main',
            'spin_in_place = xgo_lite2_driver.examples.spin_in_place:main',
        ],
    },
)
