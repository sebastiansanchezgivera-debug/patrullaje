from setuptools import setup
package_name = 'tb4_stage_simulator'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project maintainer',
    maintainer_email='maintainer@example.com',
    description='Stage-only simulated Create 3 telemetry',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'create3_sim = tb4_stage_simulator.create3_sim:main',
    ]},
)
