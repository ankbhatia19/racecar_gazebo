import os
from glob import glob
from setuptools import setup

package_name = 'racecar_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.gazebo')),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='f20171569@hyderabad.bits-pilani.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
