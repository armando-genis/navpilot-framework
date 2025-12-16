# Workspaces

The calibration uses a workspace to store the settings and the calibration results.
In this, the idea is to use one workspace per robot ([Robot Workspace](#robot-workspace)) in which different sub-workspaces ([Calibration Workspace](#calibration-workspace)) are created for each sensor and calibration type.

A typical workspace structure looks as follows:

```text
<RobotName>
|   settings.ini
|   <SomeUrdfFile>.urdf   
|   <SomeUrdfFile>_<LastDateModified>.urdf 
|
|----<src_sensor_name>_<ref_sensor_name>_<intrinsic|extrinsic>_calibration
|   |   settings.ini
|   |   calibration_results.txt
|   |   urdf_snippet.txt
|   |   <calibration_target>.yaml
|   |   
|   |---observations
|   |---_backups
|   |   |   ...
|   
|----<src_sensor_name>_<ref_sensor_name>_<intrinsic|extrinsic>_calibration
|   |   ...
|
| ...
```

The default root folder of all workspaces is ```$HOME/multisensor_calibration```

Details on the content of the different workspaces are discussed in the following.

## Robot Workspace

The root directory (i.e. the robot workspace) as well as each subdirectory (i.e. the calibration workspaces) hold a settings file (```settings.ini```) which holds the robot and calibration specific settings, respectively.

The robot settings hold the following parameters:

- ```name```: name of the robot
- ```urdf_model_path```: Optional path to URDF model of robot. When a relative path is given, it is assumed that this is relative to the robot workspace folder.

If an URDF Model is provided the extrinsic calibration results are directly written into the URDF file.
Furthermore, the extrinsic pose computed by the calibration is directly stored into the URDF file if it is available. 
In this case the existing file is renamed with the date of the last modification appended to the filename.
The calibration can still be performed, even if no urdf file is provided. 
A warning message will be emitted nonetheless. 

### Initialize New Robot Workspace

To initialize a new robot workspace there are two options:

1. Run the [Calibration Configurator](tutorial.md#calibration-configurator) and start a calibration with the parameterization of the new workspace.
The configurator will create and initialize a new workspace if it doesn't exist.

2. Alternatively, one can use the [```initialize_robot_workspace```](nodes_and_nodelets.md#initialize_robot_workspace) node. Simply run:<br>
```bash
ros2 run multisensor_calibration initialize_robot_workspace --ros-args -p robot_ws_path:=<path_to_ws_dir> -p robot_name:=<robot_name> [-p urdf_model_path:=<path_to_file>]
```

## Calibration Workspace

The calibration settings comprise the calibration specific settings and correspond to the launch parameters of the different calibration routines as stated below.
After each calibration the results are stored in a ```calibration_results.txt``` file in the corresponding subdirectory.
In addition to the calibration results, an XML block is written into ```urdf_snippet.txt``` with the calibration data which can be copied into a URDF file.

If specified in the launch configuration of the calibration nodes, the captured observations will be stored in ```observations```.
At the start of a new calibration previous calibrations will be backed-up and moved into a new subfolder within ```_backups```.

If a URDF file is specified in the robot settings and if an entry for the calibrated sensor exists in this file, the extrinsic calibration results are directly written into the URDF file.

## Workspace Templates

The [Calibration Configurator](tutorial.md#calibration-configurator) also allows to install workspace templates. 
This is especially helpful when running the calibration for the first time.
By default the calibration configurator has a workspace template for an example robot. 
If the Multi-Sensor Calibration toolbox is build and installed from source, however, one can provide the configurator with more customized workspace templates.
To do so:

1. Create a folder structure somewhere in the file system with (multiple) robot workspaces, each containing the desired calibration workspaces as depicted above.
2. Within each robot workspace folder create a 'settings.ini' file according to this [template](assets/workspace_tempaltes/robot_ws_settings_template.ini).
3. And within each calibration workspace create a 'settings.ini' file according to the appropriate template:<br>
    - [extrinsic_camera_lidar_workspace_settings_template](assets/workspace_tempaltes/extrinsic_camera_lidar_calib_ws_settings_template.ini)
    - [extrinsic_lidar_lidar_workspace_settings_template](assets/workspace_tempaltes/extrinsic_lidar_lidar_calib_ws_settings_template.ini)
    - [extrinsic_camera_reference_workspace_settings_template](assets/workspace_tempaltes/extrinsic_camera_reference_calib_ws_settings_template.ini)
    - [extrinsic_lidar_reference_workspace_settings_template](assets/workspace_tempaltes/extrinsic_lidar_reference_calib_ws_settings_template.ini)<br><br>
**NOTE:** In this it is important that the stated workspace type corresponds to the calibration type

4. Call the script within the repository called 'scripts/populate_robot_workspaces_qrc.sh' with the path to the root directory of the custom workspace structure as argument prior to building 'multisensor_calibration'.
This will copy the custom workspaces an populate the corresponding resource file so that the calibration configurator access them for installation.