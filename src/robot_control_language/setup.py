import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'robot_control_language'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # This will include all sub-packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/coffee', glob('robot_control_language/coffee/*.yaml')),
    ],
    package_data={
        'robot_control_language': [
            'coffee/settings.yaml',  # Adjust the path to ensure it matches your directory structure
        ],
    },
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
            'terminal_chat = robot_control_language.terminal_chat:main',
            'realsense_llm = robot_control_language.realsense_llm:main',
        ],
    },
)