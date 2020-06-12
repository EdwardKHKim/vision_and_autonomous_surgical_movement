# Vision and Autonomous Surgical Movement

The repository is contains algorithms in Python 2.0 split into two nodes. The vision node can recognize the object of interest using HSV values. The autonomous movement node can move the da Vinci surgical robot's PSM1 arm to the object of interest, pick up the object of interest, and move it to another location. The following diagram provides and overview of the system: 

## Computer Pre-Check 
The following repository is dependent on components that only work on Ubuntu. If your machine is running on macOS, download [Virtual Machine](https://www.virtualbox.org/) and [Ubuntu](https://ubuntu.com/download/desktop). If the speed of Ubuntu is slow, you need to allocate more RAM. 

## Development Requirements
We use the catkin build tools, NOT catkin_make. Please don't use catkin_make

#### DVRK Components 
1. Create and initialize catkin workspace named `dvrk_ws`.
```
mkdir -p ~/dvrk_ws/src
cd ~/dvrk_ws
catkin init
```
2. Download and compile the cisst libraries and SAW components for the dVRK, see the dVRK tutorial wiki: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros
3. Download and compile dvrk-ros: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

ROS has multiple releases (Hydro, Indigo, Jade...). Use the Melodic release.  

#### OpenCV 
The vision node is dependent on the OpenCV (Open Source Computer Vision Library), an open source software library for computer vision and machine learning. 
1. Install OpenCV-Python in Ubuntu: https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html

#### Coppelia Simulation
1. Download the [Coppelia Simulator](https://coppeliarobotics.com/downloads). Use the edu version with full features. Extract and cd into the root CoppeliaSim folder. 
2. Copy the `libsimExtROSInterface.so` file from the `compiledRosPlugins` folder to the root CoppeliaSim Folder. 

This system uses `dVRK-training_pick&place.ttt` scene provided in [dvrk-vrep](https://github.com/unina-icaros/dvrk-vrep). The scene is readily available in CoppeliaSim. 

#### Vision and Autonomous Surgical Movement
1. Create and initialize a catkin workspace named `vis_and_auto_mov_ws`.
```
mkdir -p ~/vis_and_auto_mov_ws/src
cd ~/vis_and_auto_mov_ws
catkin init
```
2. Clone this repository into `vision_and_auto_mov_ws`.
```
cd ~/vis_and_auto_mov_ws/src
git clone https://github.com/EdwardKHKim/vision_and_autonomous_surgical_movement
catkin config --extend /path/to/dvrk_ws
```
3. Install dependencies.
```
cd ~/vis_and_auto_mov_ws
rosdep init
rosdep install --from-path src --ignore-src -r -y
```
4. Build packages.
```
catkin build
```

## Vision Node 
The pipeline 
The following are the steps to run the vision node in the system. Each step requires a new terminal tab or window.
1. The roscore can be launched using the roscore executable:
```
roscore
```
2. Launch CoppeliaSim
```
cd /path/to/coppeliasim/root
./coppeliaSim.sh 
```
3. In CoppeliaSim: File &#8594;
## Autonomous Movement Node

## Todo
