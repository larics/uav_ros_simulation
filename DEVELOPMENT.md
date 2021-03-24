# Development Guidelines

## VSCode

### Extensions

Heavily suggested extensions are:

* xaver.clang-format
* notskm.clang-tidy
* cheshirekow.cmake-format

Install the following programs outside VSCode:
```
sudo apt install clang-tidy clang-format
```

### Setup

To ensure a smoother development experience while using Gitman and symlinked folders do the following steps:

1. After launch VSCode click ```File->Open folder``` and select the *src* folder in the catkin workspace.

2. Run the following script

```
cd $HOME/catkin_ws/src/uav_ros_simulation
./installation/unpack_modules
```
This script should unpack all the ROS packages in a more development friendly way for VSCode.
For more information plesase read the commanents in [unpack_modules.sh](installation/unpack_modules.sh)

3. Link .cmake-format.yaml to the root of your VSCode workspace
ln -s $HOME/catkin_ws/src/uav_ros_simulation/ros_packages/uav_ros_stack/.cmake-format.yaml $HOME/catkin_ws/src/.cmake-format.yaml

4. Link .clang-format.yaml to the root of your VSCode workspace 
ln -s $HOME/catkin_ws/src/uav_ros_simulation/ros_packages/uav_ros_stack/.clang-format $HOME/catkin_ws/src/.clang-format

5. Copy .vscode folder contents
cp -r $HOME/catkin_ws/src/uav_ros_stack/miscellaneous/dotvscode $HOME/catkin_ws/src/.vscode
```

4. Edit include paths in [c_cpp_properties.json](miscellaneous/dotvscode/c_cpp_properties.json) and [settings.json](miscellaneous/dotvscode/settings.json).