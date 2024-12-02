from setuptools import find_packages, setup

package_name = 'py03_tf_broadcaster'

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
    maintainer='zw',
    maintainer_email='zw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_tf_static_broadcaster_py = py03_tf_broadcaster.demo01_tf_static_broadcaster_py:main',
            'demo02_tf_dynamic_broadcaster_py = py03_tf_broadcaster.demo02_tf_dynamic_broadcaster_py:main',
            'demo03_tf_point_broadcaster_py = py03_tf_broadcaster.demo03_tf_point_broadcaster_py:main',
        ],
    },
)
