# Frequently Asked Questions

## Q: I have troubles with installing uav_ros_stack/simulation.

A: Installation should succeed on a clean Ubuntu 18.04 or 20.04 platform. No guarantees given for other environments. Please leave an issue on an appropriate repo.

## Q: I have troubles building uav_ros_stack/simulation.
A:  Check for package collisions; You might already have ROS packages that are contained in uav_ros_stack / simulation.

## Q: I have troubles running startup scripts (e.g. kopterworx_one_flying) ?
A: Check if environment is properly sourced in ~/.bashrc (or ~/.zshrc) - correct lines found in [README.md](README.md)

NOTE: Lines are automatically added to ~/.bashrc (and ~/.zshrc if exists) so if a line is missing or its wrong please report an issue!

## Mavros is not able to start with 1,500,000 Baudrate, eventhough Ardupilot officially supports it.
Thanks to [Fix: MAVROS does not support 1500000 baud rate](https://www.programmersought.com/article/95921196958/).
Solving this requires building mavros from source.

```bash
cd ~/catkin_ws/src
git clone https://github.com/mavlink/mavros
```

Before building Mavros open the following file:
```bash
sudo vim /usr/include/boost/asio/impl/serial_port_base.ipp
```

Place the following blocks in appropriate places:
```C++
# ifdef B1500000
  case 1500000: baud = B1500000; break;
# endif
```
And:
```C++
# ifdef B1500000
   case B1500000: value_ = 1500000; break;
# endif
```
Save, exit and build your Catkin workspace:
```bash
cd ~/catkin_ws/src
catkin clean
catkin build
```

## I upgraded to Ubuntu 20.04 and now I can't use Pyqtgraph backend for rqt_plot.

First install pyqtgraph through any of the methods specified [here](http://www.pyqtgraph.org/).
```bash
pip install pyqtgraph
```

Start editing the following file ```~/.local/lib/python3.8/site-packages/pyqtgraph/Qt.py```. 
**NOTE** It might be located eslewhere depending on yout pip setup.

Find all occurences of the following lines or its variations (PyQt5, PyQt6,):
```python
from PyQt5 import QtGui, QtCore, QtWidgets, uic, sip
```
and replace it as follows:
```python
try:
    # new location for sip
    # https://www.riverbankcomputing.com/static/Docs/PyQt5/incompatibilities.html#pyqt-v5-11
    from PyQt5 import sip
except ImportError:
    import sip
```

To check if it works open python command line and try importing pyqtgraph.
If successfull you are able to use the pyqtgraph backend for rqt_plot.