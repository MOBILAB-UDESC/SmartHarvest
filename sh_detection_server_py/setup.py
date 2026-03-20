from setuptools import setup
import os
from glob import glob

package_name = 'sh_detection_server_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('model/*.pt')),
    ],
    install_requires=['setuptools', 'supervision', 'ultralytics', 'opencv-python', 'numpy==1.26.4'],
    zip_safe=True,
    maintainer='nilton',
    maintainer_email='nilton.csv18@edu.udesc.br',
    description='Object Detection server in python',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sh_detection_server_py = sh_detection_server_py.detection_server:main',
        ],
    },
)
