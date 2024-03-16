# Ros2RoboticsCpp

Implement popular robotics algorithms in C++ and ROS2

* To-do-list
    - Localization:
        - [x] EKF localization
        - [x] Particle filter localization
    - Mapping:
        - [x] Lidar to grid map
        - [x] Kmeans_clustering
    - SLAM:
        - [x] EKF SLAM
        - [x] FastSLAM 
    - Path Planning
        - [x] Dynamic Window Approach
        - [x] A* 
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

- [**Particle_filter**](./src/particle_filter/)
    * Red Markers: Robot pose estimated with Particle Filter
    * Blue Markers: Ground-truth poses
    * Green Markers: Particles poses
    * Black Markers: Robot pose estimated with Dead Reckoning
    * Brown Markers: Landmarks

    ```bash
    ros2 launch particle_filter particle_filter.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/Particle_filter.gif" width="960" />
    </p>    

- [**Lidar2GridMap**](./src/lidar2gridmap/)
    * Left plot: 2D Laser scanning point cloud
    * Right plot: Occupancy grid map 

    ```bash
    ros2 launch lidar2gridmap lidar2gridmap.launch.py 
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/lidar2gridmap.png" width="960" />
    </p>   

- [**Kmeans_clustering**](./src/kmeans_clustering/)
    * Red Sphere: Object pose 
    * Cluster with different colors as different labels 

    ```bash
    ros2 launch kmeans_clustering kmeans.launch.py 
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/Kmeans.gif" width="960" />
    </p>      

- [**EKF SLAM**](./src/ekf_slam/)
    * Red Sphere: Robot pose estimated with EKF-SLAM
    * Blue Markers: Ground-truth poses
    * Black Markers: Robot pose estimated with Dead Reckoning
    * Black Rec: GT poses of Landmarks
    * Green Rec: Landmark poses estimated with EKF-SLAM

    ```bash
    ros2 launch ekf_slam ekf_slam.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/ekf_slam.gif" width="960" />
    </p>                

- [**Dynamic Window Apporach**](./src/dwa/)
    * Red Sphere: Robot pose and its traversal path
    * Green Markers: Best predicted trajectory
    * Black Markers: Obstacles
    * Yellow Markers: Goal

    ```bash
    ros2 launch dwa dwa_run.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/dwa.gif" width="960" />
    </p>                

- [**A Star From Both Sides**](./src/a_star_two_sides)
    * Green Markers: Best predicted trajectory
    * Black Markers: Environment including boundary and obstacles
    * Yellow markers: Passing nodes from star to end 
    * Green marker: Passing nodes from end to start

    ```bash
    ros2 launch a_star_two_sides a_star_two_sides.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/a_star_two_sides.gif" width="960" />
    </p>   

- [**FastSLAM 1.0**](./src/fast_slam)
    * Green Markers: Particle poses
    * Black Markers: Robot pose estimated with Dead Reckoning
    * Black Rec: Landmark poses
    * Red Markers: Robot pose estimated with FastSlam 1.0
    * Blue Markers: Robot pose GT

    ```bash
    ros2 launch fast_slam fastSlam1.launch.py
    ```

    <p align="center">
    <img src="https://github.com/quangnhat185/Media/blob/main/ros2RoboticsCpp/fastSlam.gif" width="960" />
    </p>   




## Reference
[AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)