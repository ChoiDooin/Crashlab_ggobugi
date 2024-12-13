from setuptools import find_packages, setup

package_name = 'json_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['receive_data.json']),  # Add your JSON file here
    ],
    install_requires=['setuptools','rcply'],
    zip_safe=True,
    maintainer='kgh',
    maintainer_email='kgh@todo.todo',
    description='A ROS 2 package to demonstrate reading JSON files.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'publisher = json_data.pub:main',
        ],
    },
)
