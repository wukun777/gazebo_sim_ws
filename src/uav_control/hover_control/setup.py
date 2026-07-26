from setuptools import find_packages, setup

package_name = 'hover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simulation',
    maintainer_email='3165037426@qq.com',
    description='Direct Gazebo hover controller for the roarm_quad aerial manipulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hover_node = hover_control.hover_node:main'
        ],
    },
)
