# Waypoint Saving with Particle Filter Localization

This code publishes and saves car waypoints recieved as odometry information from the Particle Filter Package. 


# Usage

The majority of parameters you might want to tweak are in the launch/localize.launch file. You may have to modify the "odometry_topic" or "scan_topic" parameters to match your environment.

```
roslaunch particle_filter localize.launch
```

 

