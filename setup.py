from setuptools import find_packages, setup

package_name = 'pointcloud_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pointcloud_object_detection_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ganesh',
    maintainer_email='gthimmankatti@gmail.com',
    description='Convert pointcloud to laserscan for obstacle and staircase detection',
    license='Sphaira Inc',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_object_detection = pointcloud_detection.pointcloud_object_detection:main',
            'pointcloud_transformer = pointcloud_detection.pointcloud_transformer:main',
        ],
    },
)
