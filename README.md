# mav_imu

Userspace driver for adis16448 and BMI088 written in C++17.

## Setup

1. Install dependencies

```shell
$ sudo apt-get install ros-noetic-imu-tools libgoogle-glog-dev
$ git clone git@github.com:ethz-asl/lpp.git
```

2. Go to the catkin workspace and clone this repo in the `src` folder and build it with
```shell
$ git clone --recurse-submodules git@github.com:ethz-asl/mav_imu.git
$ catkin build mav_imu
```
3. Source environment

```shell
$ source /opt/ros/noetic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
```

4. Launch node
```shell
$ roslaunch mav_imu imu_default.launch
```

- Optionally run with `rviz`

```shell
$ roslaunch mav_imu imu_rviz.launch
```

- Optionally run with `imu_madgwick_filter`

```shell
$ rosrun imu_madgwick_filter imu_filter_node
```

5. Example BMI088 and Jetson Xavier NX
- Connect BMI088 to 40 pin header according to [schematics](docs/Connection_BMI088_to_Jetson_Xavier_NX.pdf)
```
roslaunch mav_imu imu.launch imu:=bmi088 spi_path:=/dev/spidev0.0
```

***

There is also a kernel level driver but:

![](docs/adis16400.png)
