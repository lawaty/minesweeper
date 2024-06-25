## Workspace Setup Errors
> 1. catkin_make must be invoked in the root of workspace

This might happend due to multiple reasons:
1. You are not running the command in the root directory of the workspace
2. Workpace is not setup correctly. Make sure you run catkin_init_workpsace inside the src directory and delete workspace/CMAkeLists.txt file

> 2. no module named '\<any ros package\>'

This might happen due to multiple reasons:
1. package needs setup
2. `package.xml and CMakeLists` are not updated so that these packages are not involved in the last build or not added to exec_dependencies