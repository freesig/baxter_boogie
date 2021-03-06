# Baxter Boogie
This software is designed to make [Baxter](https://www.rethinkrobotics.com/baxter/) and [Mobility Base](http://dataspeedinc.com/resources/baxter-mobility-base/) dance to music. It works with the [Dance Detect](https://github.com/freesig/dance_detect) software

# Install instructions
- Install [Kinetic ROS](http://wiki.ros.org/kinetic/Installation)
- Install Dance Detect
- Clone this repo
- Build the package. Run `catkin_make` from ROS root. [Further Instructions](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

# User guide

### Base
- [Run the dance detect software](https://github.com/freesig/dance_detect)
- From the package root directory, run `rosrun boogie_dev move_base.py`

### Torso
- [Run the dance detect software](https://github.com/freesig/dance_detect)
- `cd <package_root_directory>`
- run `source baxter.sh` to set up a Baxter Torso environment
- run `rosrun boogie_dev move_left_inc.py & rosrun boogie_dev move_right_inc.py`
