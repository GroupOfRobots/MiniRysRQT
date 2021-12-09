from glob import glob

from setuptools import setup

package_name = 'control_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/elements'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/control_panel.ui','resource/setup.ui']),
        ('share/' + package_name + '/resource/imgs', glob('resource/imgs/*.png')),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pablo@todo.todo',
    description='Control panel for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
