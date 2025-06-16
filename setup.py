from setuptools import find_packages, setup

package_name = 'bloom_for_you'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'bloom_for_you_interfaces'  # 추가: 인터페이스 패키지 의존성
    ],
    zip_safe=True,
    maintainer='d-1',
    maintainer_email='your@example.com',
    description='Python-based ROS 2 package with custom messages.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_manager = bloom_for_you.scenario_manager:main',
            'flower_recommender = bloom_for_you.flower_recommender:main',
            'seed_planting = bloom_for_you.seed_planting:main',
            'pub_test = bloom_for_you.publisher_test:main',
            'sub_test = bloom_for_you.subscriber_test:main',
            'test = bloom_for_you.test:main',
            'wrap = bloom_for_you.flower_wrapping:main',
            'water = bloom_for_you.watering_scheduler:main',            
            'gui_test = bloom_for_you.gui_test:main',
            # 'pub_test = bloom_for_you.pub_test:main',
        ],
    },
)
