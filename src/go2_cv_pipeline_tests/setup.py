from setuptools import find_packages, setup

package_name = 'go2_cv_pipeline_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'unitree_sdk2py',
    ],
    zip_safe=True,
    maintainer='saura',
    maintainer_email='saura@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_user_test_commands = go2_cv_pipeline_tests.go2_user_test_commands:main',
            'go2_cv_test = go2_cv_pipeline_tests.go2_cv_test:main', 
        ],
    },
)
