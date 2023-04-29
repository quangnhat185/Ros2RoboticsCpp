# Ros2RoboticsCpp

Implement popular robotics algorithms in C++ and ROS2

* To-do-list
    - Localization:
        - [x] EKF localization
        - [ ] Particle filter localization
    - Mapping:
        - [ ] Lidar to grid map
        - [ ] Ray casting grid map
    - SLAM:
        - [ ] EKF SLAM
        - [ ] FastSLAM 
    - Path Planning
        - [ ] Dynamic Window Approach
        - [ ] A* 
        - [ ] Probabilistic Road-Map
        - [ ] Rapid-Exploring Random Trees (RRT)


## Localization 
- [**EKF_localization**](./src/ekf_localization/)
    * Red Markers: Robot pose estimated with EKF
    * Blue Markers: Ground-truth poses
    * Green Markers: Robot poses obtained from Observations
    * Black Markers: Robot pose estimated with Dead Reckoning

    ```bash
    ros2 launch ekf_localization ekf_launch.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/EKF_localization.gif" width="960" />
    </p>


## Reference
[AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)