from setuptools import setup
import os

package_name = 'uwb_follower'

setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lyla Ott',
    maintainer_email='lcott39@gmail.com',
    description='UWB follower node',
    license='TODO',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    entry_points={
        'console_scripts': [
            'uwb_follower = uwb_follower.uwb_follower:main',
        ],
    },
)