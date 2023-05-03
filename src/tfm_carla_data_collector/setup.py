from setuptools import setup
import os
from glob import glob

package_name = 'tfm_carla_data_collector'


def read_requirements():
    with open('requirements.txt', 'r') as f:
        return [line.strip() for line in f.readlines()]
    

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/SetDataCollection.srv'])
    ],
    install_requires=read_requirements(),
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = tfm_carla_data_collector.data_collector:main',
        ],
    },
)
