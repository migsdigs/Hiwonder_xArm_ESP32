from setuptools import find_packages, setup

package_name = 'arm_servos_pubs_subs'

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
    maintainer='miguel_u22',
    maintainer_email='miguelgarcianaude@gmail.com',
    description='Communicate with hiwonder servo arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = arm_servos_pubs_subs.move_servos_publisher:main',
            'listener = arm_servos_pubs_subs.servo_pos_subscriber:main',
        ],
    },
)
