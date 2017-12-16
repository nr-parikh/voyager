# Voyager: Explore the environment!
[![Build Status](https://travis-ci.org/nr-parikh/voyager.svg?branch=master)](https://travis-ci.org/nr-parikh/voyager)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/nr-parikh/voyager/blob/master/LICENSE)

## Contents 
* [Overview](#overview) 
* [Robot description](#description)
* [Algorithm](#algorithm) 
* [Dependencies](#dependencies)
* [Building the code](#building)
	* [Installing dependencies](#install-dependencies)
	* [Building the project](#build-project)
* [Running](#running)
* [Testing](#testing)
* [Documentation](#docs) 
* [SIP process](#sip)
* [Presentation](#present)
* [License](#lic) 

## <a name="overview"></a> Overview 
Unmanned aerial vehicles(UAV) are becoming ubiquitous in recent times. As their usage increases they need to be able to know their surrounding and be able to perceive their surroundings. There are numerous applications of UAVs like agricultural mapping, delivery systems, surveillance etc. In almost most of the
applications exploring the environment and creating a map of it to localize itself is the key part to perceive the surroundings. 

For a unmanned ground vehicles it is not a problem to have a 2-D occupancy grid but in case of UAVs it becomes mandatory to have a 3-D map of the environment. The trajectory planning of a UAVs needs a 3-D map to have a good trajectory rather than an occupancy grid. In this project, a quadrotor vehicle for security surveillance/exploring the environment. The quadrotor will be able to create a map of the environment it is in while exploring the environment. This package has a simulated robot which can explore the environment and map it without colliding with the obstacles that might be present in the surrounding. 

## <a name="description"></a> Robot description 
The quadrotor is a generic unmanned aerial vehicle of X-configuration. The robot has a laser scanner which it can use to perceive the environment around it. The task for the robot is to explore the environment and generate the map of it. The robot also has a front facing which can be used for various purposes. For the purpose of safety, the motors of the robot can be disabled with a kill switch. 

The simulated robot used in this package is from [Hector quadrotor](http://wiki.ros.org/hector_quadrotor) family of the packages by TU Darmstadt. It has a Hokuyo laser scanner which is mounted at the bottom of the robot. This laser scanner is used for detecting the presence of obstacles in the surrounding of the robot. Hector quadrotor also an Asus camera that has an ability to produce point clouds. In this project, the front facing camera of the robot is used to create a 3D map of the environment. The point cloud obtained from the camera is given to the [Octomap](http://wiki.ros.org/octomap) algorithm which creates a map of the environment. To monitor the height, the robot has a sonar sensor. The robot has a service `/enable_motors` that can be used to disarm the motors in cases of emergency. 

## <a name="algorithm"></a> Algorithm 
The algorithm used in this package is very naive. The robot has to explore the world without colliding with the environment. In order to check of the collision, the algorithm uses a readings from laser scanner. Since laser scanner gives the readings from all the directions and the readings of interest are only that occur in the from, the algorithm prunes the unnecessary data and then checks for presence of obstacle in that filtered data. If the obstacle is present then robot keeps rotating until it finds a free way to move and if there is no obstacle then it will keep moving forward. In order, to check the height of the robot from the ground it uses the readings from the sonar sensors. The algorithm constantly monitors the height of the robot and tries to maintain it to some constant value. If it goes below the threshold it thrusts up and it goes above the threshold it will try to go down. 

## <a name="dependencies"></a> Dependencies
The dependencies of this project are as given below:
* Ubuntu 16.04
* ROS Kinetic Kame 
* Catkin
* Gazebo 
* Hector ROS packages 
* Octomap ROS packages 
* Rviz 
* Teleop twist keyboard 
* Octovis 

## <a name="building"></a> Building the code   
### <a name="install-dependencies"></a> Installing dependencies
It is assumed here that ROS is already installed on the system. In order to install *hector quadrotor* packages there are two ways and can be done as follows:

Installing from binary packages:
```
$ sudo apt-get install ros-kinetic-hector-*
```

Or one can build from the source:
```
$ mkdir ~/ros_ws
$ cd ~/ros_ws
$ wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/hydro-devel/tutorials.rosinstall
$ catkin_make 
```

Then workspace needs to be source to use the project. 

To install another dependency *Octomap* following steps can be followed:
```
$ sudo apt-get install ros-kinetic-octomap
$ sudo apt-get install ros-kinetic-octomap-mapping 
$ rosdep install octomap_mapping 
$ rosmake octomap_mapping
```

The other two dependencies can be installed as follows. Navigate to the src of workspace:
```
$ git clone https://github.com/OctoMap/octomap_rviz_plugins.git
$ cd ..
$ catkin_make
```

And,
```
$ sudo apt-get install ros-kinetic-octovis
```

To use teleop package, install the following package as follows:
```
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
```


>NOTE: Both, hector and octomap_mapping, the packages are notorious for not building properly in the first go. Please ensure that both the packages are installed properly before moving forward. 

### <a name="build-project"></a> Building the package
Once all the dependencies are fulfilled, the package can be built as follows. Navigate to the src folder of the workspace:
```
$ git clone https://github.com/nr-parikh/voyager.git
$ cd ..
$ catkin_make
```

## <a name="running"></a> Running the node 
To run the node please follow the instruction given in this section. Open a terminal and launch a file using given below:
```
$ roslaunch voyager world.launch
```

The command given above will start gazebo and rviz plugin and a custom world. It will also spawn the robot in that world. To start exploring and mapping the environment, launch the following file:
```
$ roslaunch voyager voyager.launch
```

As soon as this command is launched the robot will start exploring the world unless and until it is asked to stop. In order to stop the robot and land it at a safe place, one can use a service `/explore` as follows:
```
$ rosservice call /explore false
```

This command will stop the robot from exploring and land it safely on ground. However, it should be ensure that there is a safe place for the robot to land. To start exploring again, one can call the same service but now with `true` and not `false`. 

One can also create a new custom world and use that world in this package as well by launching the same file i.e. *world.launch* but with following argument:
```
$ roslaunch voyager world.launch world:=<path to world file>/<world file>
```

Instead of having the robot to explore on its own, one can manually operate the robot and explore as one wants. This can be done using teleop package after installing it as follows:
```
$ roslaunch voyager world.launch use_teleop:=true
```

#### Recording bag file 
The bag file can also be recorded using the following command using the same launch file:
```
$ roslaunch voyager world.launch record_bag:=true
```
This command will record the bag file for the duration of 30 seconds and store it in results directory. 
>NOTE: All the topics are being recorded except the camera topics since the size of the file was off the limit. 

### Saving the tree
The tree being generated can be save by running the following command:
```
$ rosrun octomap_server octopmap_saver -f <name of the file>
```

This will save the file at the location from where the command was executed. 

## <a name="testing"></a> Testing 
The unit tests for this package can be ran using the following commands. After cloning the repository, navigate to the root of the workspace and run the following command:
```
$ catkin_make run_tests 
```

Alternatively, the tests can also be run as follows:
```
$ rostest voyager test_voyager.launch
```

#### Viewing the generated tree 
To view a tree please install octovis as mentioned in the dependencies section. To view the file execute the following command:
```
$ octovis <path to file>
```

For example, a sample map given in the results section can be viewed as:
```
$ octovis <path to ws>/voyager/results/sample_output.ot
```

## <a name="output"></a> Sample output
Sample output generated by the package by exploring the a custom test worl looks as shown below:
![alt-text](results/sample_output.png)

## <a name="docs"></a> Documentation
The documentation for this project can be found at the path `docs/html/index.html`. The documentation for this project is created using doxygen-gui. To generate the documentation again, please execute the commands given below:
```
$ sudo apt-get install doxygen 
$ sudo apt-get install doxygen-gui
$ doxywizard
```

The last command opens up the GUI window of doxygen. First select a dummy folder for doxygen to run from. After that, complete the details as required like name, synopsis, and version of the project. Select this repository as source directory and select a destination directory as well. Please make sure you check scan recursively option in order to generate the complete documentation. Once doxygen runs successfully, navigate to your destination directory, go to html folder and open index.html file.

## <a name="sip"></a> Solo Iterative Process 
While developing this module SIP process was followed. The link to the SIP sheet for this module is [here](https://docs.google.com/spreadsheets/d/11tZz-o4cJSky1bMGR0uIQGLcoGgMJUM74vtqE4XKSQ8/edit?usp=sharing). Please note that the legend for the code tags is mentioned in the sheet itself.

The spreadsheet contains product backlog which gives overview of how the project was executed. The second spreadsheet is Task backlog which gives details of how each of the task were executed and the time consumed. The thirs spreadsheet is Release backlog which shows the improvements in the estimation of the time required to complete certain task as the module is developed. 

Planning notes can be found [here](https://docs.google.com/document/d/1XSsnkajWHP6XwBxO1Zjkb_wmKyFWa5k23H9Gzb1IpiQ/edit?usp=sharing).

## <a name="present"></a> Presentation
The slides for the final presentation can be found [here](https://docs.google.com/presentation/d/1I7c-rBKHdLnsbCT6sVaDmLgEeXg7oTwyJsZUmdR4C7E/edit?usp=sharing). 

The video of the presentation as well as demo of the project can be found [here](https://youtu.be/Qogoys-AiRE).

## <a name="lic"></a> License
The license of the document can be found [here](LICENSE).