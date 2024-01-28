from glob import glob

from setuptools import setup

package_name = 'camera_video_recorder_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
              package_name + '/elements',
              package_name + '/threads'
              ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*.ui')),
        ('share/' + package_name + '/resource/imgs', glob('resource/imgs/*.png')),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paweł Rawicki',
    maintainer_email='pawel.rawicki@gmail.com',
    description='Camera video recorder on robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
