from setuptools import find_packages, setup

package_name = 'warehouse_mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/qr_model.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anany',
    maintainer_email='anany@todo.todo',
    description='todo',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'mission_executor = warehouse_mission_control.mission_executor:main',
            'qr_pipeline = warehouse_mission_control.qr_pipeline:main'
        ],
    },
)

