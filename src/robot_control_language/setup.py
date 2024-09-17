from setuptools import setup

package_name = 'robot_control_language'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'termcolor',
        'openai',
        'tenacity'
    ],
    zip_safe=True,
    maintainer='breno',
    maintainer_email='brenodeangelo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terminal_chat = robot_control_language.terminal_chat:main'
        ],
    },
)
