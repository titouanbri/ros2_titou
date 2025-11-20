from setuptools import find_packages, setup

package_name = 'ur3_traj'

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
            'traj = ur3_traj.traj:main',
            'ur3_moveit_action = ur3_traj.ur3_moveit_action:main',
        ],
    },
)
