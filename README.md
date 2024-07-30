# gstream
A program to calculate the steering angle of the wheels and estimate the trajectory of an autonomous delivery robot with Ackerman kinematic steering system.

![block_steering_ack](https://github.com/user-attachments/assets/0605c1bc-8eea-4f14-ae0d-2b0e8fa1699c)

## 1. PC Setup
### 1.1. Installing the nescessary software
Open the Terminal app and enter these commands:
```sh
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```
### 1.2. Installing package
```sh
cd <your_ws>/src
git clone https://github.com/WanL0q/gstream.git
cd ..
catkin_make
source <your_ws>/devel/setup.bash
```
## 2. Configuration
You need to calibrate the camera to determine __wb, tw__ (the corresponding number of pixels on the frame), the __start_point__ corresponding to the two wheels, the __getPerspectiveTransform__ matrix, and change the __rtsp_url__ accordingly.

## 3. Quick Start Guide
```sh
roslaunch gstream self_driving.launch
```
