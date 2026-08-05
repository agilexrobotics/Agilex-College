from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'piper_pos_slider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lansijian',
    maintainer_email='2033374848@qq.com',
    description='Slider and test publisher for Piper end-effector PosCmd control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_pos_slider = piper_pos_slider.piper_pos_slider_node:main',
        ],
    },
)
