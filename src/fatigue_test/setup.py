from setuptools import find_packages, setup

package_name = 'fatigue_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_fatigue_test.py']),  # Aggiungi i tuoi file di lancio
    ],
    install_requires=['setuptools','rclpy', 'pi3hat_moteus_int_msgs'],
    zip_safe=True,
    maintainer='JacopINO, JakoPunk',
    maintainer_email='cionix90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           
            "fatigue_jump_node = fatigue_test.fatigue_test_jump_vicone_no_bag:main",
           
           
        ],
    },
)
