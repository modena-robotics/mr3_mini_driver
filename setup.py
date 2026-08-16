from setuptools import find_packages, setup

package_name = 'mr3_mini_driver'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/plotjuggler_layout.xml'])        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matheus Lino',
    maintainer_email='matheus.vlino@gmail.com',
    description='ROS 2 driver package for the MR-3 Mini palletizing robot',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mr3_mini_driver = mr3_mini_driver.driver:main'
        ],
    },
)
