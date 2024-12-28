from setuptools import find_packages, setup

package_name = 'autorace_core_ROSticks'

setup(
    name=package_name,
    version='6.6.6',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name, [
            'package.xml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/autorace_core.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bakafox',
    maintainer_email='v.chekhovskii@g.nsu.ru',
    description='This package is a piece of shit! Imma sorry...',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = autorace_core_ROSticks.controller:main'
        ],
    },
)
