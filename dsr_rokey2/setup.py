from setuptools import find_packages, setup

package_name = 'dsr_rokey2'

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
    maintainer='rokey',
    maintainer_email='88tkeks@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_basic = dsr_rokey2.move_basic:main',
            'move_periodic = dsr_rokey2.move_periodic:main',
            'burger_robot_bridge_original = dsr_rokey2.burger_robot_bridge_original:main',
            'mini_jog = dsr_rokey2.mini_jog:main'
        ],
    },
)
