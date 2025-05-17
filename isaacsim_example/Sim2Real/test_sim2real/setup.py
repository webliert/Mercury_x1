from setuptools import find_packages, setup

package_name = 'test_sim2real'

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
    maintainer='webliert',
    maintainer_email='3474803708@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish = test_sim2real.publish:main',
            'subscriber = test_sim2real.receive_joint_information:main',
            'conrol = test_sim2real.receive_control_joint:main',
        ],
    },
)
