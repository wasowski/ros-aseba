This [ROS] stack is a bridge to access an [Aseba] network from [ROS].
For compilation instructions, see the README.md in the `asebaros` directory.

[Aseba]: http://aseba.wikidot.com
[ROS]: http://www.ros.org

Build Instructions
==================

* Install ROS (this fork is tested with kinetic)
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
