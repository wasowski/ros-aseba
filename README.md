This [ROS] stack is a bridge to access an [Aseba] network from [ROS].
For compilation instructions, see the README.md in the `asebaros` directory.

[Aseba]: http://aseba.wikidot.com
[ROS]: http://www.ros.org

Prerequisites 
=============
* You need Ubuntu or Windows with WSL (native Ubuntu works much better)


Build Instructions
==================

* Install ROS (this fork is tested with kinetic, but we are now trying to move it to melodic)
  * for Ubuntu: http://wiki.ros.org/kinetic/Installation/Ubuntu
  * for WSL: https://janbernloehr.de/2017/06/10/ros-windows (this is melodic, work in progress)
* Create a catkin workspace 
* git clone --recursive this repo into src/
* possibly also git submodules update --init --recursive
* catkin_make

Execution Instructions
======================
* roslaunch thymio_driver thymio.launch

There is also a joystick controller but I did not test it. I don't have a
joystick.

Changes from Upstream
=====================
* Move the driver to Aseba 1.5.5 and Dashel 1.3.0
* Fix generation of messages when building asebaros (build-time bug)
* Refactor to new class names and method names in Aseba, add the heartbeat
  message every 1s.
* Delay the driver initialization by a few seconds, until the node connects (a
  temporary hack)

Known Issues
============
* The driver does not reconnect after thymio has disconnected and reconnected.
  (best to restart the driver and thymio after such)
