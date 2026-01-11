from setuptools import find_packages, setup

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valerii-shvets',
    maintainer_email='valerii-shvets@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = camera_subscriber.camera_node:main',
	    'camera_point_publisher = camera_subscriber.camera_point_publisher:main',
	    'camera_point_publisher_zad_4 = camera_subscriber.camera_point_publisher_zad_4:main',
	    'zad_5 = camera_subscriber.zad_5:main',
	    'zad_6 = camera_subscriber.zad_6:main',
        ],
    },
)
