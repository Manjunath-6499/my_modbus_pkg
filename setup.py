from setuptools import find_packages, setup

package_name = 'my_modbus_pkg'

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
    maintainer='dynaspede',
    maintainer_email='dynaspede@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_comm = my_modbus_pkg.modbus_comm:main',
            'modbus1_comm = my_modbus_pkg.modbus1_comm:main',
            'modbus_comm2 = my_modbus_pkg.modbus_comm2:main',
            'modbus_comm3 = my_modbus_pkg.modbus_comm3:main',
            'modbus_comm4 = my_modbus_pkg.modbus_comm4:main',
            'modbus_comm5 = my_modbus_pkg.modbus_comm5:main',
            'modbus_comm6 = my_modbus_pkg.modbus_comm6:main',
            'modbus_comm7 = my_modbus_pkg.modbus_comm7:main',
            'modbus_comm8 = my_modbus_pkg.modbus_comm8:main',
            'modbus_comm9 = my_modbus_pkg.modbus_comm9:main',
            'modbus_comm10 = my_modbus_pkg.modbus_comm10:main',
            'modbus_comm11 = my_modbus_pkg.modbus_comm11:main',
            

        ],
    },
)
