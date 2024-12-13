from setuptools import setup
from setuptools import find_packages

package_name = 'untitled_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dohyeonse',
    maintainer_email='dohyeonse@todo.todo',
    description='Package for zone navigation',
    license='TODO',
    entry_points={
        'console_scripts': [
        ],
    },
)

