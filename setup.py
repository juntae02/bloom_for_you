from setuptools import find_packages, setup

package_name = 'bloom_for_you'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d-1',
    maintainer_email='your@example.com',
    description='Python-based ROS 2 package with custom messages.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flower_recommender = bloom_for_you.flower_recommender:main',
            'pub_test = bloom_for_you.publisher_test:main',
            'sub_test = bloom_for_you.subscriber_test:main',
        ],
    },
)
