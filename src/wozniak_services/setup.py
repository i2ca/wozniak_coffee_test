from setuptools import setup

package_name = 'wozniak_services'

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
    maintainer='breno',
    maintainer_email='brenodeangelo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_object_server = wozniak_services.pick_object:main',
            'mock_unity_server = wozniak_services.mock_unity_server:main',
        ],
    },
)
