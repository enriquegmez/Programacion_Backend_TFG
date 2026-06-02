from setuptools import find_packages, setup
import os # <--- ASEGÚRATE DE IMPORTAR OS
from glob import glob # <--- Y GLOB

package_name = 'app_tiago'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        # Le dice que dentro de app_tiago (y sus subcarpetas) copie todos los .json
        package_name: ['**/*.json'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_node = app_tiago.main:entry_point'
        ],
    },
)
