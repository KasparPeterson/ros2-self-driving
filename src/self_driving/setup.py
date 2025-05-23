from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'self_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'robot'), glob('models/robot/*')),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_driver = self_driving.self_driver:main',
            'drive = self_driving.drive_action_client:main',
        ],
    },
)
