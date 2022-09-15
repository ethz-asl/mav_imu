# mav_imu

Userspace driver for adis16448 written in C++17.

## Setup

1. Install dependencies

```shell
$ sudo apt-get install ros-noetic-imu-tools
```

2. Go to the catkin workspace and clone this repo in the `src` folder and build it with
```shell
$ git clone https://github.com/ethz-asl/mav_imu.git
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

5. Optionally run with `rviz`

```shell
$ roslaunch mav_imu imu_rviz.launch
```
***

There is also a kernel level driver but:

![](docs/adis16400.png)

