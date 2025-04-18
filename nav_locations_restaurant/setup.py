from setuptools import find_packages, setup

package_name = 'nav_locations_restaurant'

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
    maintainer='melvin',
    maintainer_email='melvin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'nav_to_named_goal = nav_locations_restaurant.nav_to_named_goal:main',
        'gui_nav_node = nav_locations_restaurant.gui_nav_node:main',
        'robot_launcher_main = nav_locations_restaurant.robot_launcher:main',

        
    ],
},
)
