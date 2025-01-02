from setuptools import find_packages, setup

package_name = 'data_recorder'

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
    maintainer='qi',
    maintainer_email='qi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "topic_recorder = data_recorder.topic_recorder:main",
            "multi_topic_recorder_vector = data_recorder.multi_topic_recorder_vector:main",
            "multi_topic_recorder_image = data_recorder.multi_topic_recorder_image:main",
            "dataset_recorder = data_recorder.dataset_recorder:main"
        ],
    },
)
