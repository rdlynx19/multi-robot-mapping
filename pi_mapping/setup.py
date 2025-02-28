import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'pi_mapping'  # Replace with your package name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),  # Corrected typo: 'launch' instead of 'launch'

        # Install all URDF and XACRO files from the 'urdf' directory
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro')),

        # Install URDF XACRO files from include into urdf/include directory
        (os.path.join('share', package_name, 'urdf', 'include'),
         glob('urdf/include/*.urdf.xacro')),
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
            # Add your console scripts here (if any)
        ],
    },
)
