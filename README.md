# Team Skynet - self-driving car system integration

## TODO: Short Project Description

Self Driving car basic system architecture
![general-system-arch](imgs/system_arch.png)

## CARLA

__Mission:__ Control CARLAs throttle, steering and brakes to succefully navigate map waypoints.

### CARLAs ROS Nodes:

1. Perception
    
    * Traffic light detection
    * Obstacle detection

2. Planning

    * Waypoint Updater - sets target velocity for each waypoint based on upcomming traffic lights and obstacles.

3. Control subsystems

    * DBW (Drive By Wire) - takes target trajectory information as input and sends control commands to navigate the vehicle.

![carla-ros-graph-v2](imgs/carla-ros-graph-v2.png)

## TODO: The Team

__Nauris Dorbe:__ Machine Learning Expert at [SqualioCC](http://squaliocc.com/en), Researcher assistant at [EDI](http://edi.lv/en/home/) and Doctor candidate at [University of Latvia](https://www.lu.lv/eng/). Interested in Machine Learning and Self Driving cars. [Github](https://github.com/Naurislv). [LinkedIn](https://www.linkedin.com/in/naurisdorbe). [Twitter](https://twitter.com/NaurisDorbe)

__Neil Maude__ (<neil.maude@gmail.com>): General Manager at [Arena Group EDM](http://www.arenagroup.net/).  MA (Math/Computing), University of Oxford. MBA, Warwick Business School. Interested in AI, Machine Learning, Data Visualisation and Self Driving Cars/Autonomous Vehicles. [Github](https://github.com/NeilMaude). [LinkedIn](https://www.linkedin.com/in/neilmaude). [Twitter](https://twitter.com/nmaude2006).

__Martin Liesenberg:__ Software Engineer at [BCG Digital Ventures](https://bcgdv.com/). MsC in Computer Science from FU Berlin. Interested in and working with machine learning, stream processing and autonomous vehicles. [Github](https://github.com/mliesenberg)

__Alexander Noll:__ Data Scientist at [PartnerRe](http://partnerre.com/). PhD in Mathematics from University of Vienna, MSc in Engineering Physics from TU Graz. Interested in machine learning, statistics, artificial intelligence and self-driving cars. [Github](https://github.com/NOllAl)

__Jonas Biehler:__ Research Engineer at AdCo Engineering. Phd in Computational Mechanics. Interested in Simulation, AI, Machine Learning, HPC, and self-drinving cars. [Github](https://github.com/jbi35), [LinkedIn](https://www.linkedin.com/in/jonas-biehler-82138a9a/)

# Original repo README

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

