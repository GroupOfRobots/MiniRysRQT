from glob import glob

from setuptools import setup

package_name = 'shared'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/test', package_name + '/deleted_robot_screen', package_name + '/utils',  package_name + '/stack_widget', package_name + '/base_widget'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('*/*.json')),
        ('share/' + package_name + '/data/robots', glob('data/robots/*.json')),
        ('share/' + package_name + '/resource', ['resource/deleted_robot.ui']),
        ('share/' + package_name + '/resource/imgs', glob('resource/imgs/*.png'))
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
