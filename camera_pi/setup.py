from setuptools import find_packages, setup

package_name = 'camera_pi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ← ADD THIS: installs your launch files
        ('share/' + package_name + '/launch',
            ['launch/hazard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Youssef Shemela',
    maintainer_email='Youssefshemela12@gmail.com',
    description='Ballbot hazard detection and camera streaming nodes',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ← FIXED: was "camer_node" (missing 'a')
            'camera_node = camera_pi.camera_streaming:main',
            'hazard_detector = camera_pi.hazard_detector:main',
        ],
    },
)
