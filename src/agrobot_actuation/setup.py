from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_actuation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steff',
    maintainer_email='san63@byu.edu',
    description='Seed and Removal nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'remove = agrobot_actuation.remove:main', # executable name ran = python import path + function
		'seed = agrobot_actuation.seed:main',
		'button_client = agrobot_actuation.button_client:main',
        ],
    },
)
