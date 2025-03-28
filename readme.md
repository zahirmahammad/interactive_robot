```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find turtlebot3_manipulation_gazebo)/models
```

## Mobile Manipulator

### Setup - Turtlebot Side

1. Create a map

    ```bash
    # launch gmapping node
    roslaunch turtlebot3_slam turtlebot3_gmapping.launch
    # OR
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

    # visualize
    roslaunch turtlebot3_manipulation_navigation navigation.launch
    ```

2. Store the map

    ```bash
    rosrun map_server map_saver -f ~/map
    ```

3. Store the main points [Waypoints]
    ```bash
    rosrun llm_robot save_pose.py
    ```


### Manipulator Side

1. Write some pre-defined functions for controlling the manipulator

    - Pickup
    - Deliver



### Files to launch - after mapping

1. Launch the Simulation + Load the map

    ```
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
    ```

2. Start the LLM Script that controls the robot