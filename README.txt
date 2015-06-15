/=================================================================================/
/                       README file RF_RIDDLE Ros package                         /
/                                                                                 /
/  Created by        : Arthur Valiente (CNRS-LAAS intern 2015)                    /
/  Contact           : valiente.arthur@gmail.com                                  /
/                                                                                 /
/  History :                                                                      /
/       Person       |       Date       |              Modification               /
/---------------------------------------------------------------------------------/
/  Arthur Valiente   |    15/06/2015    |         Creation of the file            /
/                                                                                 /
/                                                                                 /
/                                                                                 /
/                                                                                 /
/                                                                                 / 
/=================================================================================/



		DESCRIPTION

  This package has been created for the RIDDLE project at CNRS-LAAS of Toulouse.
His goal is to provide a ROS module to be able to communicate with the RF electronics system created at the LAAS.
Therefore, this packages can communication through UART/I2C (TO BE DEFINED) with the RF system to configure it or start an RF acquisition.

This package define few topics/services to be able to use and configure it with another ROS module. 



		INSTALLATION GUIDE (/!\ catkin & git require /!\)

    Dependances & ROS packages needed for installation :
      ROS :
       -> RVIZ
       -> Roscpp
       -> Rospy
      other stuff :
       -> None

    How to install:

  1- First, setup your workspace for installation.
Create or use an already existing catkin workspace wherever you want.

To create a new workspace (recommended) follow the instructions:
 ->Go where you want to put your catkin workspace and type the following commands on your terminal:
mkdir -p rf_riddle_ws/src
cd rf_riddle_ws/src
catkin_init_workspace

 -> You can build your workspace even if it is empty by doing the following:
cd rf_riddle_ws
catkin_make

 -> Within the <your_ws_name>/src/ folder, create a new directory, corresponding to the rf package where you will put all sources files:
cd rf_riddle_ws/src
mkdir rf_riddle && cd rf_riddle

 -> Don't forget to source the setup.bash file from catkin
source /path-to-your-folder/rf_riddle_ws/devel/setup.bash

 2- Download the sources files. Inside your "package" folder, git clone the sources files:
git clone https://github.com/ArthurVal/RF_detection_Stage_RF_OR.git

 3- Compile the ROS node:
cd ../.. (go to rf_riddle_ws/ )
catkin_make



		UTILISATION

  The RF node can be run in 2 differents ways.

First you can run it in normal mode (the node will create all ROS stuff and will try to setup the UART. It will shutdown if the UART port isn't connected to the RF card).
You can run this mode simply by launching le node using this command:
rosrun rf_riddle rf_riddle_node

The second mode is a stub mode. It doesn't use UART but simulate the data from the rf system.
You can run this mode thanks to the "--stub" option:
rosrun rf_riddle rf_riddle_node --stub

You can run this node with other options:
 --remote
 --thetaDisable
 --verbose

Check --help option to know what they are doing.
rosrun rf_riddle rf_riddle_node --help



		TECHNICAL INFORMATION

Automatic mode max speed: 10Hz (loop_rate ros speed)

node name:    RF_detection_node

topics used:  
 - visualization_marker (<visualization_msgs::Marker>)
 - rf_riddle_intensity_map_topic (<rf_riddle::RF>)

services:
 - rf_riddle_intensity_map_srv (ask for running a rf detection wit input param given)
 - rf_riddle_set_param (set params for rf acquisiiton but don't run detection. Can be used in automatic mode)
















