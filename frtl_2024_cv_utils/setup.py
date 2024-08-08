from setuptools import setup, find_packages

package_name = 'frtl_2024_cv_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='python_modules'),
    package_dir={'': 'python_modules'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for computer vision utilities.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Make sure the name of the executable matches your intended usage and the actual file
            'yolo_classifier = yolo_classifier:yolo_classifier.main',
        ],
    }
)
