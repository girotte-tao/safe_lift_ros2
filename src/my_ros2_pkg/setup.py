from setuptools import setup

package_name = 'my_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='taoware@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_ros2_pkg.publisher:main',
            'subscriber = my_ros2_pkg.subscriber:main',
            'ws_publisher = my_ros2_pkg.ws_publisher:main'
        ],
    },
)
