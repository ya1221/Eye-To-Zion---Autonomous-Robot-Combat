import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ai_audio'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), [
            'models/impact_cnn.onnx',
            'models/labels.json',
            'models/feature_stats.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eye-To-Zion',
    maintainer_email='todo@todo.com',
    description='Eye-To-Zion audio pipeline: mic capture, impact-trigger detection, onboard classification',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
            'audio_processor_node = ai_audio.audio_processor_node:main',
        ],
    },
)
