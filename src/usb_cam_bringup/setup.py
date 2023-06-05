from setuptools import setup
import os
from glob import glob

package_name = 'usb_cam_bringup'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    package_dir={'': 'launch'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leekai',
    maintainer_email='antondel00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
