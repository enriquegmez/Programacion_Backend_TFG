import os
from glob import glob
from setuptools import find_packages, setup # type: ignore

package_name = 'r2pilot_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        # Incluye todos los esquemas JSON de validación en la instalación del paquete
        package_name: ['**/*.json'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala los archivos de lanzamiento (launch) para que ROS 2 los registre
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrique Gómez',
    maintainer_email='enriquegmez@correo.ugr.es',
    description='Backend asíncrono y puente ROS 2 para el sistema R2Pilot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_node = r2pilot_server.main:entry_point'
        ],
    },
)