from setuptools import setup

package_name = 'robocolumbus25_stuff'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robocolumbus 1/6 scale jeep wheel controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocolumbus25_wheel_controler_node = robocolumbus25_stuff.robocolumbus25_wheel_controler_node:main'
        ],
    },
)