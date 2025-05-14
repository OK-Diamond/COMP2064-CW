# GP Waiting Room Manager

Team 16 - Oliver Kirk, James Coward, Matthew Miles

Authorship:
grove_modules - Matt
ros_ws - James
web_pages - Oliver

## USERS

Run the flask server via `web_pages/main.py`

## DEVELOPERS

To install python libraries,
run `pip install -r requirements.txt`

### ROS launch

#### Prerequisites

- A map file of the area
- To have created locmap location data (see [locmap](#locmap))
- To Edit the core.launch file with your map file location under the map_file arg

#### Building

- Go to the ros_ws directory
- Run `catkin_make` (or `catkin build` if you have the build tools)
- Run `source devel/setup.bash` (still in the ros_ws directory)

#### Running

- Run `roslaunch lodestone core.launch`

### SIM launch

#### Prerequisites

Unlike the core launch the sim launch has no prerequisites assuming you want to use the same world and map 
that has already been generated

#### Building

- Go to the ros_ws directory
- Run `catkin_make` (or `catkin build` if you have the build tools)
- Run `source devel/setup.bash` (still in the ros_ws directory)

#### Running

- Run `roslaunch lodestone simcore.launch`

### Files

#### Lodestone

Lodestone is the central ROS node it is responsible for collecting data from the other nodes and MQTT broker
and directing the robot through some states.

Lodestone has an internal state IDLE -> TO_WAITING -> WAITING_AT_PATIENT -> TO_GP -> WAITING_AT_GP
it progresses linearly through these states starting idle until both a gp becomes free and a patient
registers. Lodestone moves through states like TO_WAITING and TO_GP by tracking the status of the move_base
package e.g. when the robot has completed its last action the status of move_base will be SUCCEEDED, 
during navigation it would be ACTIVE. After completing the navigation lodestone can move the robot to the 
next state i.e. WAITING_AT_PATIENT.

#### Locmap

Locmap is an abstraction over map locations. It has two modes record and publish;
in record it will listen for goal events and then prompt for a name, this means that you can load the navigation
node with your desired map and then using rviz place goals that are then given a name. 
Publish is used to both publish the locations that are available and provide a goto functionality where a
node can publish the named location and locmap will direct the robot to its map position.

#### Bugeyes

Bugeyes is a debug ui that allows you to simulate sensor inputs. For example by sending button
MQTT events, fake USR data, or new patient data being registered. Bugeyes also displays all of the logs generated
by ROS nodes using dropplot/logger

#### Dropplot

Dropplot is a collection of shared functions and data. It provides shared functionality such as the logger and
encoding/decoding datatypes. Along with a data types file for patients and pairings.

### Logging

ROS nodes are recommended to use the logging provided in dropplot/logger.py, logs sent
via the log_(info/warn/err) are published to /dropplot/logs which are displayed by
bugeyes.
