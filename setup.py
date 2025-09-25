from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'measure_reflect_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuukikawai-ubuntu22',
    maintainer_email='24rmd16@ms.dendai.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = measure_reflect_distance.my_node:main',
            'unet_inference_node = measure_reflect_distance.unet_inference_node:main',
            'sync_camera_info = measure_reflect_distance.sync_camera_info:main',
            'mirror_plane_estimation = measure_reflect_distance.mirror_plane_estimation:main',
            'tct_calibrator = measure_reflect_distance.tct_calibrator:main',
            'tag_real_static_broadcaster = measure_reflect_distance.tag_real_static_broadcaster:main',
        ],
    },
)
