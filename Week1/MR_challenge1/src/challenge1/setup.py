from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'challenge1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brunene',
    maintainer_email='A01798275@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = challenge1.signal_generator:main',
            'process = challenge1.process:main',
        ],
    },
)
