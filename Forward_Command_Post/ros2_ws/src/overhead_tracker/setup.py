from glob import glob

from setuptools import find_packages, setup

package_name = 'overhead_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yahav',
    maintainer_email='yahav990@gmail.com',
    description='Overhead ArUco-based robot and target tracking node for the Forward Command Post (FCP) system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = overhead_tracker.overhead_tracker:main',
        ],
    },
)
