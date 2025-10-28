## 2025 ESDA Vehicle Simulation
### 1. Set up
#### 1.1. Install Necessary Dependencies

#### 1.2. Building the Project
- Please clone the repository into the `src` folder of your work space, your folder structure should look like this:
```
|_workspace_name
    |_src
        |_2025_esda_simulation
```
- When you are trying to build the project, please do it in the `\workspace_name` level. So the `log`, `build` and `install` folders are generated next to the `src` folder. 
```
|_workspace_name
    |_src
        |_2025_esda_simulation
    |_install
    |_build
    |_log
```
- To source your local build into the current path, so it can be seen by ros2.
```
source install/setup.bash
```