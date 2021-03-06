from setuptools import setup

package_name = 'ski_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Artur Coelho',
    maintainer_email='artur.temporal@hotmail.com',
    description='Goes around stuff',
    license='Apache 2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ski = ski_controller.ski:main'
        ],
    },
)
