from glob import glob

from setuptools import setup

package_name = 'shared'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
              package_name + '/alert',
              package_name + '/spinner',
              package_name + '/publishers',
              package_name + '/subscription_dispatcher',
              package_name + '/deleted_robot_screen',
              package_name + '/no_robot_configuration_screen',
              package_name + '/utils',
              package_name + '/stack_widget',
              package_name + '/services',
              package_name + '/base_plugin',
              package_name + '/base_widget'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('*/*.json')),
        ('share/' + package_name + '/data/robots', glob('data/robots/*.json')),
        ('share/' + package_name + '/resource', ['resource/deleted_robot.ui']),
        ('share/' + package_name + '/resource', ['resource/no_robot_configuration.ui']),
        ('share/' + package_name + '/resource/imgs', glob('resource/imgs/*.png'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paweł Rawicki',
    maintainer_email='pawel.rawicki@gmail.com',
    description='Control panel for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
