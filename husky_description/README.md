# Husky Description
In order to facilitate editing structures, payloads, and edit other features of the Husky, we use the following environment variables. All of the definitions below are of the default values for each variable.

## Namespace
When dealing with multiple Husky robots it is necessary to set a namespace for each
```bash
export ROBOT_NAMESPACE='/'
```

## File Paths
The following variables are used to set the paths of extra configuration files. 
```bash
# Extend URDF with custom file
export HUSKY_URDF_EXTRAS='empty.urdf'
export CPR_URDF_EXTRAS='empty.urdf'
```

## Bumpers
The front and rear bumpers can be toggled on/off and extended using the following variables. The front bumper can be swapped for one with wibotic wireless charging.
```bash
export HUSKY_FRONT_BUMPER=1
export HUSKY_REAR_BUMPER=1
export HUSKY_FRONT_BUMPER_EXTEND=0
export HUSKY_REAR_BUMPER_EXTEND=0
export HUSKY_WIBOTIC_BUMPER=0
```

## Top Plate
By default, the standard plate is added atop the user rail. This top plate can be swapped for a larger top plate.

**Standard Plate:**
```bash
export HUSKY_TOP_PLATE_ENABLED=1
export HUSKY_LARGE_TOP_PLATE=0
export HUSKY_USER_RAIL_ENABLED=1
```

**Large Top Plate:**
```bash
export HUSKY_TOP_PLATE_ENABLED=1
export HUSKY_LARGE_TOP_PLATE=1
export HUSKY_USER_RAIL_ENABLED=0
```


## PACS
The PACS system overwrites the previous top plate and user rail with a custom top plate with preset locations. 
There are 56 standard locations organized in a grid, from A01 to G08 (where A through G define the columns, while 1 through 8 define the rows. The middle value is the height level. Additional levels can be added using risers).
```bash
# Enable to use swap to PACS top plate
export HUSKY_PACS_ENABLED=1
```

### Risers
There are two types of risers that can be added to the robot. \
Full risers span the entire top plate (i.e. it is like adding a second plate atop the default one). Only one can be added.\
Partial risers span a row of the plate. Multiple can be added. \
To add multiple, enter a list of **LEVELS** and **ROWS**. These lists must be of the same length. The **LEVELS** list defines the height of each riser; while, the **ROWS** list defines the row of each riser. \

```bash
export HUSKY_FULL_RISER_LEVEL=1 # 10cm increment
export HUSKY_PARTIAL_RISER_LEVELS="1 5" # List of levels, 10cm and 50cm
export HUSKY_PARTIAL_RISER_ROWS="1 8" # List of rows, first row (at 10 cm), second row (at 50 cm)
```

### Brackets
Once you have enabled PACS (and any risers), you have a three dimensional grid atop of which you can add brackets.
There are three types of brackets:
- **horizontal**: standard bracket, a 10cm x 10cm plate.
- **horizontal_large**: larger 13cm x 13cm plate.
- **vertical**: base is same size as standard plate, but has a vertical wall where mounting link is added.

These brackets can be added to any hardpoint (i.e. A01 through G08). In the following environment variables 'A01' is used as an example but it can be swapped for any hardpoint (including those on risers).
```bash
export HUSKY_A01_MOUNT_ENABLED=1
export HUSKY_A01_MOUNT_TYPE='horizontal' # or 'horizontal_large' or 'vertical'
export HUSKY_A01_MOUNT_XYZ='0 0 0'
export HUSKY_A01_MOUNT_RPY='0 0 0'
export HUSKY_A01_MOUNT_EXTENSION='0' # distance from surface of plate/riser to surface of bracket
```

## Sensor Arch
The standard sensor arch can be added using the following environment variables. 
```bash
export HUSKY_SENSOR_ARCH=1
export HUSKY_SENSOR_ARCH_HEIGHT='510' # or 300
export HUSKY_SENSOR_ARCH_OFFSET='0 0 0'
export HUSKY_SENSOR_ARCH_RPY='0 0 0'
```

# Husky Payloads
The following environment variable provide a system to add standard sensors to the Husky URDF. \
Every sensor has three types of environment variables: 
1. **Enable**: these will add the sensor to the URDF and start the launch file. 
2. **Launch**: these correspond to parameters exclusively in the launch file.
3. **Description**: these correspond to parameters excluvesively in the URDF. 

## SICK LMS1XX Laser
#### Primary
```bash
export HUSKY_LMS1XX_ENABLED=1
```
###### Launch
```bash
export HUSKY_LMS1XX_TOPIC='front/scan'
export HUSKY_LMS1XX_IP='192.168.131.20'
```
###### Description
```bash
export HUSKY_LMS1XX_PREFIX='front'
export HUSKY_LMS1XX_PARENT='top_plate_link'
export HUSKY_LMS1XX_XYZ='0.2206 0.0 0.00635' # standard offset when parent is 'top_plate_link'
export HUSKY_LMS1XX_RPY='0.0 0.0 0.0' # standard orientation when parent is 'top_plate_link'
```
#### Secondary
```bash
export HUSKY_LMS1XX_SECONDARY_ENABLED=1
```
###### Launch
```bash
export HUSKY_LMS1XX_SECONDARY_TOPIC='rear/scan'
export HUSKy_LMS1XX_SECONDARY_IP='192.168.131.21'
```
###### Description
```bash
export HUSKY_LMS1XX_SECONDARY_PREFIX='rear'
export HUSKY_LMS1XX_SECONDARY_PARENT='top_plate_link'
export HUSKY_LMS1XX_SECONDARY_XYZ='-0.2206 0.0 0.00635' # standard offset when parent is 'top_plate_link'
export HUSKY_LMS1XX_SECONDARY_RPY='0.0 0.0 3.14159' # standard orientation when parent is 'top_plate_link'
```
## Hokuyo UST10
#### Front
```bash
export HUSKY_UST10_ENABLED=1
```
###### Launch
```bash
export HUSKY_UST10_TOPIC='front/scan'
export HUSKY_UST10_IP='192.168.131.20'
```
###### Description
```bash
export HUSKY_UST10_PREFIX='front'
export HUSKY_UST10_PARENT='top_plate_link'
export HUSKY_UST10_XYZ='0.2206 0.0 0.00635' # standard offset when parent is 'top_plate_link'
export HUSKY_UST10_RPY='0 0 0'
```
#### Rear
```bash
export HUSKY_UST10_SECONDARY_ENABLED=1
```
###### Launch
```bash
export HUSKY_UST10_SECONDARY_TOPIC='rear/scan'
export HUSKY_UST10_SECONDARY_IP='192.168.131.21'
```
###### Description
```bash
export HUSKY_UST10_SECONDARY_PREFIX='rear'
export HUSKY_UST10_SECONDARY_PARENT='top_plate_link'
export HUSKY_UST10_SECONDARY_XYZ='-0.2206 0.0 0.00635' # standard offset when parent is 'top_plate_link'
export HUSKY_UST10_SECONDARY_RPY='0 0 3.14159' # standard orientation when parent is 'top_plate_link'
```

## Velodyne VLP16
#### Primary
```bash
export HUSKY_LASER_3D_ENABLED=1
```
###### Launch
```bash
export HUSKY_LASER_3D_TOPIC='points'
export HUSKY_LASER_3D_HOST='192.168.131.20'
```
###### Description
```bash
export HUSKY_LASER_3D_TOWER=1
export HUSKY_LASER_3D_PREFIX=''
export HUSKY_LASER_3D_PARENT='top_plate_link'
export HUSKY_LASER_3D_XYZ='0 0 0'
export HUSKY_LASER_3D_RPY='0 0 0'
```
#### Secondary
```bash
export HUSKY_LASER_3D_SECONDARY_ENABLED=1
```
###### Launch
```bash
export HUSKY_LASER_3D_SECONDARY_HOST='192.168.131.21'
export HUSKY_LASER_3D_SECONDARY_TOPIC='secondary_points'
```
###### Description
```bash
export HUSKY_LASER_3D_SECONDARY_TOWER=1
export HUSKY_LASER_3D_SECONDARY_PREFIX='secondary_'
export HUSKY_LASER_3D_SECONDARY_PARENT='top_plate_link'
export HUSKY_LASER_3D_SECONDARY_XYZ='0 0 0'
export HUSKY_LASER_3D_SECONDARY_RPY='0 0 -3.1415' # standard orientation to face backwards
```
## Intel Realsense
#### Primary
```bash
export HUSKY_REALSENSE_ENABLED=1
```
###### Launch
```bash
export HUSKY_REALSENSE_SERIAL='0'
export HUSKY_REALSENSE_TOPIC='realsense'
export HUSKY_REALSENSE_POINTCLOUD_ENABLED=1
export HUSKY_REALSENSE_DEPTH_ENABLED=1
export HUSKY_REALSENSE_DEPTH_FRAMERATE='30'
export HUSKY_REALSENSE_DEPTH_HEIGHT='480'
export HUSKY_REALSENSE_DEPTH_WIDTH='640'
export HUSKY_REALSENSE_COLOR_ENABLED=1
export HUSKY_REALSENSE_COLOR_FRAMERATE='30'
export HUSKY_REALSENSE_COLOR_HEIGHT='480'
export HUSKY_REALSENSE_COLOR_WIDTH='640'
export HUSKY_REALSENSE_PREFIX=camera
```
###### Description
```bash
export HUSKY_REALSENSE_PREFIX='camera'
export HUSKY_REALSENSE_PARENT='top_plate_link'
export HUSKY_REALSENSE_XYZ='0 0 0'
export HUSKY_REALSENSE_RPY='0 0 0'
```
#### Secondary
```bash
export HUSKY_REALSENSE_SECONDARY_ENABLED=1
```
###### Launch
```bash
export HUSKY_REALSENSE_SECONDARY_SERIAL='0'
export HUSKY_REALSENSE_SECONDARY_TOPIC='realsense_secondary'
export HUSKY_REALSENSE_SECONDARY_POINTCLOUD_ENABLED=1
export HUSKY_REALSENSE_DEPTH_ENABLED=1
export HUSKY_REALSENSE_DEPTH_FRAMERATE='30'
export HUSKY_REALSENSE_DEPTH_HEIGHT='480'
export HUSKY_REALSENSE_DEPTH_WIDTH='640'
export HUSKY_REALSENSE_COLOR_ENABLED=1
export HUSKY_REALSENSE_COLOR_FRAMERATE='30'
export HUSKY_REALSENSE_COLOR_HEIGHT='480'
export HUSKY_REALSENSE_COLOR_WIDTH='640'
export HUSKY_REALSENSE_SECONDARY_PREFIX='secondary_camera'
```
###### Description
```bash
export HUSKY_REALSENSE_SECONDARY_PREFIX='camera_secondary'
export HUSKY_REALSENSE_SECONDARY_PARENT='top_plate_link'
export HUSKY_REALSENSE_SECONDARY_XYZ='0 0 0'
export HUSKY_REALSENSE_SECONDARY_RPY='0 0 0'
```
## Flir Blackfly S
#### Primary
```bash
export HUSKY_BLACKFLY=1
```
###### Launch
```bash
export HUSKY_BLACKFLY_SERIAL='0'
export HUSKY_BLACKFLY_DEVICE='USB3'
export HUSKY_BLACKFLY_ENCODING='BayerRG8'
export HUSKY_BLACKFLY_FRAMERATE='30'
```
###### Description
```bash
export HUSKY_BLACKFLY_MOUNT_ENABLED=1
export HUSKY_BLACKFLY_MOUNT_ANGLE='0'
export HUSKY_BLACKFLY_PREFIX='blackfly'
export HUSKY_BLACKFLY_PARENT='top_plate_link'
export HUSKY_BLACKFLY_XYZ='0 0 0'
export HUSKY_BLACKFLY_RPY='0 0 0'
```
#### Secondary
```bash
export HUSKY_BLACKFLY_SECONDARY=1
```
###### Launch
```bash
export HUSKY_BLACKFLY_SECONDARY_SERIAL='0'
export HUSKY_BLACKFLY_SECONDARY_DEVICE='USB3'
export HUSKY_BLACKFLY_SECONDARY_ENCODING='BayerRG8'
export HUSKY_BLACKFLY_SECONDARY_FRAMERATE='30'
```
###### Description
```bash
export HUSKY_BLACKFLY_SECONDARY_MOUNT_ENABLED=1
export HUSKY_BLACKFLY_SECONDARY_MOUNT_ANGLE='0'
export HUSKY_BLACKFLY_SECONDARY_PREFIX='blackfly_secondary'
export HUSKY_BLACKFLY_SECONDARY_PARENT='top_plate_link'
export HUSKY_BLACKFLY_SECONDARY_XYZ='0 0 0'
export HUSKY_BLACKFLY_SECONDARY_RPY='0 0 0'
```
## IMU 
These could be either Microstrain, UM7, or UM6
```bash
export HUSKY_IMU_XYZ='0.19 0 0.149'
export HUSKY_IMU_RPY='0 -1.5708 3.1416'
export HUSKY_IMU_PARENT='base_link'
export HUSKY_IMU_PORT='/dev/microstrain ' # or '/dev/clearpath/um7' or '/dev/clearpath/um6'
# for UM6 and UM7 only
export HUSKY_MAG_CONFIG="$(catkin_find 'husky_bringup' --first-only)/config/mag_config_default.yaml"
```

## Navsat 
```bash
export HUSKY_NAVSAT_PORT=/dev/clearpath/gps
export HUSKY_NAVSAT_BAUD=19200
```
