from setuptools import find_packages, setup
import glob

package_name = 'ui_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + "/ui/", glob.glob('ui_pkg/monitor.ui')),
        ('share/' + package_name + "/ui/", glob.glob('ui_pkg/odm.ui')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyum',
    maintainer_email='gyum@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = ui_pkg.monitor:main',
            'odm = ui_pkg.odm:main',
        ],
    },
)
