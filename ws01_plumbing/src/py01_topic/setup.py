from setuptools import find_packages, setup

package_name = 'py01_topic'

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
            'demo01_talker_str_py = py01_topic.demo01_talker_str_py:main',
            'demo02_listener_str_py = py01_topic.demo02_listener_str_py:main',
            'demo03_talker_stu_py = py01_topic.demo03_talker_stu_py:main',
            'demo04_listener_stu_py = py01_topic.demo04_listener_stu_py:main'
        ],
    },
)
