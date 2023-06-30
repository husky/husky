Ouster Integration
==================

Installations:
--------------
- Add ouster package into `catkin_ws/src`
    ```
    git submodule add https://github.com/ouster-lidar/ouster-ros.git
    source /opt/ros/noetic/setup.bash
    cd ../.. # back to /catkin_ws
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
Running:
--------
- First run `source devel/setup.bash` (ensure that the it's the devel in the catkin_ws directory)
- LiDAR must be connected then run:
    ```
    roslaunch ouster_ros sensor.launch 
    sensor_hostname:=os1-122011000244.local
    metadata:=<json file name>          # optional
    ```
- to see the topics being published, open another terminal, source the same bash file then run `rostopic list`


Fast-LIO Integration
====================

Installations:
--------------
1.  `sudo apt install libpcl-dev` (this is for Point Cloud Library)
2.  `sudo apt uninstall libeigen3-dev` **(UNVERIFIED)**
    - Alternatively, download tar.gz file from [Eigen source](https://eigen.tuxfamily.org/index.php?title=Main_Page) 
    - Untar using `tar -xzf <tarfile name>`
    - See instructions in `INSTALL.txt` in the untarred folder
3. Intall livox_ros driver *(TO-DO: See if this part can be removed from the source code)*
    - First install [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) and follow installation instructions in `README.md`
    - Next, add livox driver using `git submodule add https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src` to main repo
    - Run the following
        ```
        cd ws_livox
        catkin_make
        ```
4. Go back to `catkin_ws/src`
    - Add Fast-LIO package using `git submodule add git clone https://github.com/hku-mars/FAST_LIO.git`
    - do the following:
        ```
        cd FAST_LIO
        git submodule update --init
        cd ../..
        source ../ws_livox/devel/setup.sh
        catkin_make
        ```
