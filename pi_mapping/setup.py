from setuptools import find_packages, setup

package_name = 'pi_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/depthai.launch.py',
                                   'launch/stereo_inertial_node.launch.py',
                                   'urdf/base_descr.urdf.xacro',
                                   'urdf/depthai_descr.urdf.xacro',
                                   'urdf/include/base_macro.urdf.xacro',
                                   'urdf/include/depthai_macro.urdf.xacro',
                                   ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itadori',
    maintainer_email='davepushkar02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
