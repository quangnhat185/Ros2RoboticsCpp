from setuptools import find_packages, setup

package_name = 'reeds_shepp_planning'

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
    maintainer='quang',
    maintainer_email='lqnnguyen@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reeds_shepp_client = reeds_shepp_planning.reeds_shepp_client_node:main',
            'reeds_shepp_service = reeds_shepp_planning.reeds_shepp_service_node:main',
        ],
    },
)
