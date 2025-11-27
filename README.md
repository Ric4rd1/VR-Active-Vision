#  🤖🌱VR-based Active Vision for Agricultural Robotics 🌱🤖

This project explores the use of **Virtual Reality (VR)** to teleoperate a **robotic arm equipped with a stereo camera**.  
The VR headset (HMD) position and orientation controls the perspective of the camera mounted on the robotic arm in real time, enabling an **active vision system**.  
The application is oriented towards **agricultural environments**, addressing challenges such as occlusion from leaves or branches, and improving **telepresence, usability, and perceptual information comprehension**.

## ⚙️ Technologies and Hardware
- **Meta Quest 3** (VR headset) connected to Unity via  
  - [Meta Quest Link](https://www.meta.com/en-gb/help/quest/1517439565442928/)  
  - [Meta SDK](https://developers.meta.com/horizon/downloads/package/meta-xr-sdk-all-in-one-upm/)  
- **Unity 2022.3** connected to **ROS 2** using the [ROS–TCP–Connector](https://github.com/Unity-Technologies/Unity-Robotics-Hub). 
- **ROS2 Humble**
- **UFactory lite6 robot** using the [Xarm_ros2 package](https://github.com/xArm-Developer/xarm_ros2)
- **Intel Realsense D405**  using the [ROS Wrapper for RealSense](https://github.com/realsenseai/realsense-ros)


## 🚀 Setup Instructions
For setup and installation details, please refer to the dedicated guide:  
👉 [Setup Instructions](https://docs.google.com/document/d/169PzqmcX6txEb10fj3BbERi2wmETntBMpJT48AJlcgg/edit?usp=sharing)

# 🖥️ System Architecture & How to Run the VR–Active Vision System

This project requires **two machines** working together on the same network:

- **Windows PC** → Runs **Unity** + VR headset streaming + ROS–TCP client.  
- **Ubuntu 22.04 machine** → Runs **ROS 2 Humble**, the robotic arm driver, the ROS-TCP endpoint, and the camera node.

A third device, the **UFactory Lite6 robot**, must also be connected to the same LAN.  
Using a **wired Ethernet connection** for all three (Windows, Ubuntu, and Lite6) is **highly recommended** for low latency and stability.

---

## 🐧 Ubuntu Setup (ROS 2 Humble)

### 1. Create a ROS 2 workspace
```bash
mkdir -p ~/vr_active_ws/src
cd ~/vr_active_ws
```
### 2. Clone the repository
```bash
mkdir -p ~/vr_active_ws/src
cd ~/vr_active_ws/src
git clone -b main https://github.com/Ric4rd1/VR-Active-Vision.git
```

Inside this workspace, you will find three important ROS 2 packages:

#### ROS–TCP–Endpoint 

**Original repo:** https://github.com/Unity-Technologies/ROS-TCP-Endpoint  
Handles the TCP communication between Unity and ROS 2.

---

#### xarm_ros2 

**Original repo:** https://github.com/xArm-Developer/xarm_ros2  
ROS 2 drivers for the UFactory Lite6 robot, including trajectory execution.

---

#### ulite6_move 
Developed package that translates headset pose into real robot motion, enabling the active vision teleoperation behavior.

##  Windows Setup (Unity + VR)

### Requirements
- **Unity 2022.3**
- **Clone this repo** using the `unity-project` branch
- Install Unity packages listed in the *Setup Instructions*  
  *(Meta XR SDK, ROS–TCP–Connector, etc.)*
- Install **Meta Link** app and connect your **Meta Quest 3**  
  *(also described in the Setup Instructions doc)*

---

### Steps

#### 1. Clone the Unity project
```bash
git clone -b unity-project https://github.com/Ric4rd1/VR-Active-Vision.git
```

#### 2. Open the project in Unity.

#### 3. Configure the Ubuntu machine IP
Inside Unity, open **ROS–TCP–Connector** settings and set the IP address of the Ubuntu machine running ROS 2.

#### 4. Connect the Meta Quest 3
Use Meta Quest Link (wired highly recommended) or AirLink (wireless).

#### 5. Press Play in Unity
You should now see the greenhouse scene in VR.

## ▶️ Launching the Entire System (Step-by-Step)

Below is the recommended order for starting all modules.

---

## 🐧 On the Ubuntu ROS 2 machine

### 1. Source your workspace
```bash
cd ~/vr_active_ws
source install/setup.bash
```
### 2. Start the ROS–TCP Endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<UBUNTU_IP>
```
#### 3. Start the Lite6 robot driver
```bash
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=<ROBOT_IP>
```
#### 4. Start the Intel RealSense D405 camera node
```bash
ros2 run realsense2_camera realsense2_camera_node
```
## On the Windows machine

#### 5. Connect your Meta Quest 3 via **Meta Link**

#### 6. Open and run the Unity project

You should now see the robotic scene / greenhouse VR interface.

## 🐧 Final step (Ubuntu): Start head tracking → robot motion

This node converts the head pose from Unity into robot arm movement.

```bash
ros2 run ulite6_move xyzrTrack_servo
```

🎉 The system is now fully operational. Moving your head will update the robot-mounted stereo camera in real time.
## Videos 🎥
https://youtu.be/p88Vtxx2Qdg 

---
