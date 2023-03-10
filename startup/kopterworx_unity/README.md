# Kopterworx Unity

Fly the Kopterworx UAV in the Unity simulator.

## Instructions

```bash
git clone https://github.com/lmark1/AirSim
cd AirSim
./setup.sh
./build.sh
cd Unity
./build.sh
```

## Startup

Before running the session make sure to adjust paths in  ```unity_setup.sh```
as needed.

If everything is configured correctly the Tmuxinator session should launch the UnityEditor 
with the project *UnityDemo* and the scene *DroneScene*. As soon as the editor launch the simulation should start.

```bash
./start.sh
```

**NOTE** If the UnityEditor simulation plays before the ```sim_vehicle``` SITL script is running there might be connection issues.
To avoid this problem simply increase the wait time before the Unity simulator start in ```session.yml```.

## TODO
- [ ] Add Kopter model to Unity
- [ ] Add Unity installation to instructions