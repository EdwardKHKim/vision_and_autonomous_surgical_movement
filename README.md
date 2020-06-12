# Vision and Autonomous Surgical Movement

The repository contains algorithms in Python 2.0, split into two nodes. The vision node can recognize the object of interest using HSV values. The autonomous movement node can move the da Vinci surgical robot's PSM1 arm to the object of interest, pick up the object of interest, and move it to another location. The following diagram provides and overview of the system: 

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
#### What does this node do?
The node 
#### Function specifications
#### Running Vision Node
The following are the steps to run the vision node in the system. A new terminal can be a new terminal tab or window.
1. In terminal, roscore can be launched using the roscore executable:
```
roscore
```
2. In a new terminal, launch CoppeliaSim
```
cd /path/to/coppeliasim/root
./coppeliaSim.sh 
```
3. Complete the following steps in CoppeliaSim: File &#8594; Open scene &#8594; `dvrk-vrep` &#8594; `V-REP_scenes` &#8594; `dVRK-training_pick&place.ttt`
4. Click Start/resume simulation.
5. In a new terminal, run `image_flipper.py` that converts the image messages to ROS conventions.
```
cd 
cd vis_and_auto_mov_ws
source devel/setup.bash
rosrun simulation image_flipper.py
```
6. In a new terminal, run the vision node 
```
cd 
cd vis_and_auto_mov_ws
source devel/setup.bash
rosrun featurization vision.py 
```
7. A new window will appear with 
## Autonomous Movement Node
#### What does this node do?
The node 
#### Function specifications
#### Running Autonomous Movement Node
1. In terminal, roscore can be launched using the roscore executable:
```
roscore
```
2. In a new terminal, launch CoppeliaSim
```
cd /path/to/coppeliasim/root
./coppeliaSim.sh 
```
3. Complete the following steps in CoppeliaSim: File &#8594; Open scene &#8594; `dvrk-vrep` &#8594; `V-REP_scenes` &#8594; `dVRK-training_pick&place.ttt`
4. Click Start/resume simulation.
5. In a new terminal, run the dVRK console. 
```
cd
cd vis_and_auto_mov_ws
source devel/setup.bash
rosrun dvrk_robot dvrk_console_json -j console-PSM1_KIN_SIMULATED.json
```
6. The dVRK console will appear. Select _Direct Controle_. The arms can be set to any values using the _Desired position (deg)_ fields. For this system, leave the joint values as:
- _PSM1_: 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
7. In a new terminal, run `image_flipper.py` that converts the image messages to ROS conventions.
```
cd 
cd vis_and_auto_mov_ws
source devel/setup.bash
rosrun simulation image_flipper.py
```
8. In a new terminal, run the autonomous movement node.
```
cd
cd vis_and_auto_mov_ws
source devel/setup.bash
rosrun ecm_controller movement.py
```
## Todo

## Troubleshooting
- When using a code editor (Visual Studio Code, IntelliJ IDEA...), error messages in import statements can be ignored.
- If there are any questions or comments related to this repository, please contact me at edwardk.kim@hotmail.com.
