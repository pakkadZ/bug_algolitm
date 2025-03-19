from setuptools import setup

package_name = 'turtlebot_bug2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Bug 2 Algorithm for TurtleBot3',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bug2 = turtlebot_bug2.bug2:main',
        ],
    },
)
