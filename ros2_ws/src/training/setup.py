from setuptools import setup

package_name = 'training'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Publisher, subscriber, service server and client nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_publisher   = training.person_publisher:main',
            'person_subscriber  = training.person_subscriber:main',
            'service_server     = training.service_server:main',
            'service_client     = training.service_client:main',
        ],
    },
)
