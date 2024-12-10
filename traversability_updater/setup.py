from setuptools import find_packages, setup

package_name = 'traversability_updater'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/net.py']),
        ('lib/' + package_name, [package_name + '/ground_analyzer.py']),
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
