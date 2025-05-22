from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_turtlebot3_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    (os.path.join('share', package_name), ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group05-f24',
    maintainer_email='83145130+michaelreda20@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
