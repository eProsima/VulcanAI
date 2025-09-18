from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'TODO'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all JSON config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Carlos Ferreira',
    author_email='carlosferreira@eprosima.com',
    maintainer='Carlos Ferreira',
    maintainer_email='carlosferreira@eprosima.com',
    description='TODO.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'TODO = TODO.main:main',
        ],
    },
)
