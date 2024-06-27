## Workspace Setup Errors
> 1. catkin_make must be invoked in the root of workspace

This might happend due to multiple reasons:
1. You are not running the command in the root directory of the workspace
2. Workpace is not setup correctly. Make sure you run catkin_init_workpsace inside the src directory and delete workspace/CMAkeLists.txt file

> 2. no module named '\<any ros package\>'

This might happen due to multiple reasons:
1. package needs setup
2. `package.xml and CMakeLists` are not updated so that these packages are not involved in the last build or not added to exec_dependencies

## Using Python Modules inside ROS Packages
This section covers some project organization scenarios which are:
1. I need to import classes from subdirectories into my node located at the package root directory
2. I need to use define node inside a subdirectory instead of placing it in the root directory
3. I need to import classes easily into my node found at subdirectory without easily without dealing with relative paths
4. I need to import classes even from outside of the package

### Steps
1. create __init__.py file inside every directory you need to access as python module 
2. create setup.py file at the root directory of the package with the following content
```py
from setuptools import setup, find_packages

setup(
    name='descriptive name for the module',
    version='0.0.0',
    packages=find_packages(include=['directory1', 'directory2']),
    package_dir={'': ''} # root dir may be 'scripts' or anything else if you put your code in a subdirectory inside the package
)
```

3. Inside CMakeLists.txt add the following
```
  catkin_python_setup() # before messages section

  messages belongings here

  catkin_package(def here...)

install(DIRECTORY src/package_name/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)
```


now directory1.file.class is importable from all packages.