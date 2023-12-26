from setuptools import setup

package_name = 'hapManager'

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
            'publisher = hapManager.publisher:main',
            'subscriber = hapManager.subscriber:main',
            'ws_publisher = hapManager.ws_publisher:main',
            'livox_subscriber = hapManager.livox_subscriber:main',
            'subscribe_manage = hapManager.subscribe_manage:main'
        ],
    },
)
