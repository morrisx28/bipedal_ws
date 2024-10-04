from setuptools import setup

package_name = 'balance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'balance.module'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' +package_name, ['launch/balanceAllInOne.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serialpid_v1_2 = balance.serialpid_v1_2:main',
        ],
    },
)
