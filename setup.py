from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gpt_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='joshuaknight@nevada.unr.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt_node = gpt_listener.gpt_node:main',
            'gpt_talker = gpt_listener.talker:main',
            'gpt_stt = gpt_listener.listener:main',
        ]
    },
)
