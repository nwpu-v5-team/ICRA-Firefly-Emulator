# ICRA RoboMaster AI Challenge 2020 Emulator

This repository is an emulator for 2020 DJI ICRA Robomaster AI Challenge, created by Northwestern Polytechnical University Firefly team.

## Introduction

According to the rules of the 2020 ICRA RoboMaster AI Challenge competition, we have built a simulation platform based on Gazebo which simulates the real Robomaster AI robot scene.

The simulation platform we built has the following features:

- **Added support for outpose cameras in the 2020 new competition rules**. Users can obtain images of outpose cameras installed in the four corners of the venue, and can customize the camera's pose and camera internal parameters. This function provides convenience for the development of camera-related algorithms, for example, it can provide the ground truth of the poses of the robot on the field to evaluate the accuracy of the robot localization algorithm based on the outpose camera;

  <img src="https://cdn.nlark.com/yuque/0/2020/png/2394508/1598472319108-1db5c106-92b0-4077-9292-f05635a45146.png?x-oss-process=image%2Fresize%2Cw_1294" width="50%" height="50%">

- **Import the full SolidWorks model of the real robot into Gazebo**, and color the armor plate, taillights, etc., this can provide the basis for the vision testing module during simulation, and achieving the combination of vision-navigation-strategy in the simulation debugging phase, to approximate the real robot competition scene;

  <img src="https://cdn.nlark.com/yuque/0/2020/png/1802887/1598491040553-636b0a5d-5734-4322-a9f5-95a1b3247c25.png?x-oss-process=image%2Fresize%2Cw_1492" width="50%" height="50%">

- **Equipped with an IMU and a camera with depth module on the robot**, which can obtain RGB format color images, 16-bit depth images and IMU information, which facilitates the verification of target detection algorithms and the depth estimation algorithms during the simulation. Meanwhile, it provides support for the robot's shooting action;

  <img src="https://cdn.nlark.com/yuque/0/2020/png/2377550/1598473851121-f0d8ea98-3d95-41f2-9b2d-5505d680ddb0.png" width="50%" height="50%">
   
- **Support 2D Lidar and many other sensor modules to obtain information about the environment**. In the simulation scene, the same data source (such as lidar and camera) as the real robot is used as the input of the navigation and preception module. This conducives to more convenient migration of path planning and preception algorithms to real robots;

  <img src="https://cdn.nlark.com/yuque/0/2020/gif/2398843/1598428603849-4d17a35c-99eb-4d9c-adf8-c354b98d63a2.gif" width="50%" height="50%">

- **Provide a reinforcement learning interface** (please refer to our [ICRA-RoboMaster-2020-Strategy](https://github.com/nwpu-v5-team/ICRA-RoboMaster-2020-Strategy) Repository).

- **Well extendiability**. Gazebo has rich plug-ins and community ecology, which can extend our simulation platform more conveniently in future development. Everyone can custom their favorite one.

## Build && Run

Operation System：Ubuntu 18.04/16.04

ROS：Melodic/Kinetic

### Install Dependencies

Please modify to your own ROS version.

```shell
sudo apt-get install ros-melodic-joy                 \
                     ros-melodic-map-server          \
                     ros-melodic-amcl                \
                     ros-melodic-move-base           \
                     ros-melodic-controller-manager  \
                     ros-melodic-cv-bridge           \
                     ros-melodic-gazebo-ros-pkgs     \
                     ros-melodic-gazebo-ros-control  \
                     ros-melodic-ros-control         \
                     ros-melodic-ros-controllers
```

### Build

Build the ROS packages.

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/nwpu-v5-team/ICRA-Firefly-Emulator.git
cd ..
catkin_make
source devel/setup.bash # Change to `source devel/setup.zsh` if you use zsh.
```

### Run

Launch all of the four robots:

```shell
roslaunch infantry2020 simulation_2020.launch
```

If you only want to run only robot:

```shell
roslaunch infantry2020 simulation_red2.launch
```

Then the Red2 robot will be launched.

## The publish and subscribe topics

The namespace name before different topics means different robot: red1, red2, blue1, blue2. Follows are of an example of red2:

- Laser scan: `/red2/scan`

- RGB Camera: `/red2/camera_rgb/image_raw`

- RGB-D Camera:
  - Depth frame: `/red2/camera_depth/depth/image_raw`
  - RGB frame: `/red2/camera_depth/rgb/image_raw`

- Outpost Camera: `/outpost_camera/outpost_camera0/image_raw` # the outpost_camera0-3 means the different outpost camera in the field.

- The ground truth pose of robot: `: /red2/ground_truth/state`

- Odometry information: `/red2/odom`

- IMU information: `/red2/imu`

- Joint information of the robot (from this you can get the angle of the gun, etc.): `/red2/joint_states`

## Keyboard control node

Start the keyboard control node:

```shell
rosrun infantry2020 keyboard_ctrl_node __ns:=/red2
```

Notes:
• Start a keyboard control node if you want to control a single car;
• There are two underscores in front of ns, followed by the name of the namespace. For example, to control the red2 car, write __ns:=/red2;

### Instructions

| Keyboard Keys | Operation | Keyboard Keys | Operation |
| :---: | :---: | :---: | :---: |
| `w` | Forward speed increased by 0.1 | `W` | Clear forward speed |
| `s` | Backward speed increased by 0.1 | `S` | Clear backward speed |
| `a` | Left speed increased by 0.1(Panning) | `A` | Clear left speed
(Panning) |
| `d` | Right speed increased by 0.1(Panning) | `D` | Clear right speed
(Panning) |
| `q` | Left speed increased by 0.1 | `Q` | Clear left speed |
| `e` | Right speed increased by 0.1 | `E` | Clear right speed |
| `o` | Clear the speed in all directions | `c` | exit |


## Q&A

- If the  function package gazebo_ros_control is missing, ensure that the previous dependencies are completely installed;

- If there are problems such as missing msgs, you can first compile the roborts_msgs package separately;
  
  ```shell
  catkin_make -DCATKIN_WHITELIST_PACKAGES="roborts_msgs"
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```
  > Note:  When compiling again, pay attention to reset the compiled parameters to empty, otherwise there will be a problem that only one package has been compiled.

- If you want to move an object in the simulation environment, click the second cross-shaped button in the function bar, and then the object you want to move will appear with a three-axis coordinate, then you can drag the object; press and hold an axis, you can only change the size of this axis. Remember to click back to the first one controlled by the mouse when not in use to prevent accidental touch;
  
  <img src="https://cdn.nlark.com/yuque/0/2020/png/2377550/1598475690439-656e907e-637a-497f-a73b-0372452c3040.png" width="50%" height="50%">
  
- If you want to directly modify the coordinates of the object in gazebo, click the required model under the models tab under the world tab on the left (be careful not to select only one link), and then modify the data of the pose tab below;
   
  <img src="https://cdn.nlark.com/yuque/0/2020/png/2377550/1598475724827-621a3c3c-3aa4-401c-b592-e5ec94a7d934.png" width="25%" height="25%">

- If the robotmodel cannot be loaded normally, like:
  
  <img src="https://cdn.nlark.com/yuque/0/2020/png/2377550/1598475849861-ddcbb02a-45dc-4b69-9845-d5166a623ead.png" width="=10%" height="10%">
  
- You can modify TF Prefix to the required namespace, like:
  
  <img src="https://cdn.nlark.com/yuque/0/2020/png/2377550/1598475925692-883d3c0d-a943-45bb-9bab-0163fafec84e.png?x-oss-process=image%2Fresize%2Cw_650" width="25%" height="25%">
  
- Modified ground_plane. If you encounter problems that cannot be loaded, first put the icra_ground_plane folder in the ICRA-Firefly-Emulator/wall-2019/ directory under ~/.gazebo/models/.
  
  ```shell
  cd ICRA-Firefly-Emulator/wall-2020/
  cp -r icra_ground_plane/ ~/.gazebo/models/
  ```
