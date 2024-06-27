from setuptools import setup, find_packages

setup(
    name='control',
    version='0.0.0',
    packages=find_packages(include=['actuators', 'drivers', 'tests']),
    package_dir={'': ''}
)