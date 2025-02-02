from setuptools import find_packages, setup

package_name = 'raspbot'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = raspbot.motors:main',
            'sonar = raspbot.sonar:main',
            'camera = raspbot.camera:main',
            'keyboard = raspbot.keyboard:main',
            'imu = raspbot.imu:main',
            'camera_listener = raspbot.camera_listener:main',
            'apriltag = raspbot.apriltag:main',
            'ir = raspbot.ir:main',
            'buzzer = raspbot.buzzer:main'
        ],
    },
)
