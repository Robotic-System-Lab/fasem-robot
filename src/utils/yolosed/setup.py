from setuptools import find_packages, setup

package_name = 'yolosed'

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
    maintainer='lamp',
    maintainer_email='salamp@salamp.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation = yolosed.segmentation:main', # YOLO Segmentation
            'dst = yolosed.detectsegment:main', # YOLO Detection and Segmentation
            'watch = yolosed.watch:main', # Watchdog
        ],
    },
)