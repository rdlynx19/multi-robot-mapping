from setuptools import find_packages, setup

package_name = 'remote_rtabmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/remote.launch.py',
                                   'launch/rt.launch.py',
                                   'launch/remote.launch.py',
                                   'launch/custom_rgbd.launch.py',
                                   'launch/stereo_inertial_node.launch.py',
                                   'config/map.rviz',
                                   'urdf/base_descr.urdf.xacro',
                                   'urdf/depthai_descr.urdf.xacro',
                                   'urdf/include/base_macro.urdf.xacro',
                                   'urdf/include/depthai_macro.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redhairedlynx',
    maintainer_email='pushkardave.vnit@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
