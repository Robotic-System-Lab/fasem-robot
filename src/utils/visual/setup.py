from setuptools import find_packages, setup

package_name = 'visual'

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
            'tkinter = visual.tknode:main', # Visualize Segmentation
            'kivy = visual.kvnode:main', # Visualize Segmentation with Kivy
            'qt = visual.qtnode:main', # Visualize Segmentation with QT
            'vid = visual.qtvid:main', # Convert PNGs to MP4 with QT
        ],
    },
)
