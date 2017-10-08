# Team Skynet - self-driving car system integration

__Mission:__ Control CARLAs (self driving car) throttle, steering and brakes to successfully navigate map waypoints and to detect traffic lights, stopping if the light is red.

Before running our code on CARLA we developed it to work in a simulator. The simulator works in a very similar way to CARLA as all ROS nodes and topics are then same.  So basically if our system works in the simulator, we expect that it should also work on CARLA.  This is with the exception of obstacles (e.g. traffic light) detection, as different classification model is used because of pixel inputs. To start using this repo please review first [this section](#project-details).

## Running Skycar

When the simulator, ROS and catkin environment are up and running, you should  be able to run this project.

1. Navigate to repository
2. Navigate to ros directory: `cd ros`

3. Run

   * Simulator:  `./run_skycar_styx`
   * CARLA (site): `./run_skycar_site`

## Self Driving car basic system architecture. Principle behind systems architecture.

![general-system-arch](imgs/system_arch.png)

## CARLA

### CARLAs ROS Nodes:

1. Perception

    * Traffic light detection - We use a deep neural net to detect if the upcoming
     traffic light is red or not. We trained the classifier once with images from the simulator
     and once with real images from the ROS bag. A detailed description of the architecture and training parameters can be found in the respective notebooks [here](./object_detection/KerasClassificator.ipynb)
     and [here](./object_detection/KerasClassificatorUdacity.ipynb).

2. Planning

    * Waypoint Updater - sets target velocity for each waypoint based on upcoming traffic lights and obstacles. More detailed documentation can be found [here](./waypoint_updater_docs.md).

3. Control subsystems

    * DBW (Drive By Wire) - takes target trajectory information as input and sends control commands to navigate the vehicle. More detailed documentation can be found [here](./controller_docs.md).

![carla-ros-graph-v2](imgs/carla-ros-graph-v2.png)

## The Team

|     Image              |     Role      |      Name      |    Location   | LinkedIn    |     email   |
|------------------------|---------------|----------------|---------------|-------------|-------------|
| <img src="./imgs/nauris_dorbe.jpg" alt="Nauris Dorbe" width="150" height="150"> |__Lead__| Nauris Dorbe | Latvia, Riga | [Nauris](https://www.linkedin.com/in/naurisdorbe) | <naurisdorbe@gmail.com> |
| <img src="./imgs/neilmaude.jpg" alt="Neil Maude" width="150" height="150">| | Neil Maude | UK, Leeds | [Neil](https://www.linkedin.com/in/neilmaude)| <neil.maude@gmail.com> |
| <img src="./imgs/jonasbiehler.jpg" alt="Jonas Biehler" width="150" height="150">| | Jonas Biehler | Germany, Munich  | [Jonas](https://www.linkedin.com/in/jonas-biehler-82138a9a/) |<biehler.jonas@gmail.com> |
| | | Martin Liesenberg | Germany, Berlin | - | <martin.liesenberg@gmail.com> |
| | | Alexander Noll | Austria, Vienna | - | <alexander.noll.a@gmail.com> |

__Nauris Dorbe:__ Machine Learning Expert at [SqualioCC](http://squaliocc.com/en), Researcher assistant at [EDI](http://edi.lv/en/home/) and Doctor student at [University of Latvia](https://www.lu.lv/eng/). Interested in Machine Learning and Self Driving cars. [Github](https://github.com/Naurislv). [LinkedIn](https://www.linkedin.com/in/naurisdorbe). [Twitter](https://twitter.com/NaurisDorbe)

__Neil Maude:__ (<neil.maude@gmail.com>): General Manager at [Arena Group EDM](http://www.arenagroup.net/).  MA (Math/Computing), University of Oxford. MBA, Warwick Business School. Interested in AI, Machine Learning, Data Visualisation and Self Driving Cars/Autonomous Vehicles. [Github](https://github.com/NeilMaude). [LinkedIn](https://www.linkedin.com/in/neilmaude). [Twitter](https://twitter.com/nmaude2006).

__Martin Liesenberg:__ Software Engineer at [BCG Digital Ventures](https://bcgdv.com/). MsC in Computer Science from FU Berlin. Interested in and working with machine learning, stream processing and autonomous vehicles. [Github](https://github.com/mliesenberg)

__Alexander Noll:__ Data Scientist at [PartnerRe](http://partnerre.com/). PhD in Mathematics from University of Vienna, MSc in Engineering Physics from TU Graz. Interested in machine learning, statistics, artificial intelligence and self-driving cars. [Github](https://github.com/NOllAl)

__Jonas Biehler:__ Research Engineer at AdCo Engineering. Phd in Computational Mechanics. Interested in Simulation, AI, Machine Learning, HPC, and self-drinving cars. [Github](https://github.com/jbi35), [LinkedIn](https://www.linkedin.com/in/jonas-biehler-82138a9a/)

# Project details

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


### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```
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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))

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
5. Confirm that traffic light detection works on real life images
