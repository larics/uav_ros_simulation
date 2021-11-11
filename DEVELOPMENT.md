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

4. Edit include paths in [c_cpp_properties.json](miscellaneous/dotvscode/c_cpp_properties.json) and [settings.json](miscellaneous/dotvscode/settings.json).


## Gitman

When a new commit is proven to be stable in one of the subpackages, the procedure to update gitman is as follows (assuming it's one of the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack) modules):

```bash
# Navigate to uav_ros_stack module
cd ~/catkin_ws/src/uav_ros_simulation/ros_packages/uav_ros_stack

# Lock the commit ID of the desired package
# If locking fails that means changes are made in the repository, please stash them first!
gitman lock [SOME_PACKAGE]

# Carefully commit the updated commit ID in gitman.yml file (e.g. using git gui)
git gui
git push origin main

# Now we need to update uav_ros_simulation/gitman.yml
# Navigate to uav_ros_simulation
cd ~/catkin_ws/src/uav_ros_simulation

# Lock the commit ID of the uav_ros_stack
# If locking fails that means changes are made in the repository, please stash them first!
gitman lock uav_ros_stack

# Carefully commit the updated commit ID in gitman.yml file (e.g. using git gui)
git gui
git push origin main
```

Same idea works for uav_ros_simulation modules, but only one gitman.yml update is needed.

## Starting out

* Learn tmux keybinds by heart! Start from [HOWTO.md](https://github.com/larics/uav_ros_simulation/blob/main/HOWTO.md).
* If you just want to see the UAV fly go to the [startup](https://github.com/larics/uav_ros_simulation/tree/main/startup) folder and execute the ```./start``` script.
* If you have a specific use case you want to try out, copy one startup folder e.g. ```kopterworx_one_flying``` to your project and add additional nodes or programs in the ```session.yml```.
* Check out optional dependancies for [simulation](https://github.com/larics/uav_ros_simulation#optional-dependencies) and [real-world](https://github.com/larics/uav_ros_general/#optional-dependancies).