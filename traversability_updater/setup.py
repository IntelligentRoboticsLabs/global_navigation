from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'traversability_updater'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        #('share/ament_index/resource_index/packages',
        #    ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'checkpoints/VanillaVAE/version_0'), 
         glob(os.path.join('checkpoints/VanillaVAE/version_0', '*'))),
        (os.path.join('share', package_name, 'checkpoints/VanillaVAEH/version_0'), 
         glob(os.path.join('checkpoints/VanillaVAEH/version_0', '*'))),
        (os.path.join('share', package_name, 'checkpoints/VanillaVAERGBH/version_0'), 
         glob(os.path.join('checkpoints/VanillaVAERGBH/version_0', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='migueldm',
    maintainer_email='midemig@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'traversability_updater_node = traversability_updater.traversability_updater_node:main'
        ],
    },
)