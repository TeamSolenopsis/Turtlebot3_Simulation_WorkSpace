# Turtlebot3_Simulation_WorkSpace

## Prerequisite
- Gazebo installed
- ROS Humble installed

## Using
first export the turtlebot3 model
```
cd turtle_ws
export GAZEBO_MODEL_PATH=$PWD/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle
```

launch an empty gazebo world
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

spawn a single robot
```
ros2 run gazebo_ros spawn_entity.py -entity <entity_name> -robot_namespace <namespace> -file $GAZEBO_MODEL_PATH/turtlebot3_waffle/model.sdf -x <x> -y <y> -z 0.01
```

## Reference
[Turtlebo3 simulation install](https://medium.com/@thehummingbird/ros-2-mobile-robotics-series-part-1-8b9d1b74216)


