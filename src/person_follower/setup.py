from setuptools import find_packages, setup
from glob import glob

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mpriore',
    maintainer_email='mpriore@todo.todo',
    description='TurtleBot4 Person Following',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = person_follower.perception_node:main',
            'tracking = person_follower.tracker_node:main',
            'control = person_follower.controller_node:main',
            'agent = person_follower.agent_node:main',
        ],
    },
)
