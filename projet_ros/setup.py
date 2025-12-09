from setuptools import find_packages, setup

package_name = 'projet_ros'

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
    maintainer='titouan',
    maintainer_email='titouan.briancon@sigma-clermont.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'puck_detector = projet_ros.puck_detector:main',
            'position_base_control = projet_ros.position_base_control:main',
        ],
    },
)
