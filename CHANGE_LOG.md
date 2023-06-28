Ouster Integration
==================

cloned ouster repo into catkin_ws
- First run `source devel/setup.bash` (ensure that the it's the devel in the catkin_ws directory)
- LiDAR must be connected then run:
    ```
    roslaunch ouster_ros sensor.launch 
    sensor_hostname:=os1-122011000244.local
    metadata:=<json file name>          # optional
    ```
- to see the topics being published, open another terminal, source the same bash file then run `rostopic list`
