from setuptools import setup

package_name = 'slope_layer'

setup(
    name=package_name,
    version='0.0.0',
    packages=['slope_layer'],  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LylaOtt',
    maintainer_email='lcott@gmail.com',
    description='Nav2 slope costmap layer (Python prototype)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slope_costmap_node = slope_layer.slope_costmap_node:main',  
        ],
    },
)
