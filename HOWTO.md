# Simulation Startup Tutorial

## TMUX Keybinds

Running *start.sh* will automatically start and attach you to a tmux session, so it's useful to know tmux keybinds. 
The standard TMUX command prefix is **Ctrl+b**.

| Description |  Keybind |
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| Run tmux | :$ tmux |
| List tmux sessions | :$ tmux ls |
| Attach to a session | :$tmux a -t [session name] |
| New Window | **prefix**+c |
| Move through windows (tabs) | Shift+&larr; &nbsp; Shift+&rarr;|
| Killing window | **prefix**+x |
| Killing session | **prefix**+k |
| Detaching from a session | **prefix**+d |
| Enter scroll mode | **prefix**+[ |
| Split pane horizontally |  **prefix**+% |
| Split pane vertically | **prefix**+" |

## ROS Topics and Services

Move UAVs by publishing on the following topic:
```bash
rostopic pub /red/tracker/input_pose   # Namespace prefix either red, blue or yellow
```

Additional useful MAVProxy commands can be found at [ardupilot_gazebo/README.md/Simulation and Mavproxy Commands](https://github.com/larics/ardupilot_gazebo/blob/larics-master/README.md#simulation-and-mavproxy-commands).