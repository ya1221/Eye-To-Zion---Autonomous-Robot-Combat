import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ai_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models', 'best_ncnn_model'), [
            'models/best_ncnn_model/model.ncnn.bin',
            'models/best_ncnn_model/model.ncnn.param',
            'models/best_ncnn_model/metadata.yaml',
            'models/best_ncnn_model/model_ncnn.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eye-To-Zion',
    maintainer_email='todo@todo.com',
    description='Eye-To-Zion vision pipeline: capture, inference, threat assessment',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
            'cv_processor_node = ai_vision.cv_processor_node:main',
            'ai_inference_node = ai_vision.ai_inference_node:main',
        ],
    },
)