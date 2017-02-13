# PR2 Controller Framework #

This repository stores the packages used to support manipulation experiments in a PR2 robot. It implements a set of cartesian controllers that share a common joint controller. The controllers run in the PR2 realtime loop and expose an actionlib interface to allow for remote execution. 

## Packages in this repository ##

* **pr2_algorithms** : Includes the executed task level manipulation algorithms 
* **pr2_cartesian_clients** : Tools and nodes for coordinating manipulation experiments from an external machine
* **pr2_cartesian_controllers** : Realtime loop controllers that embed the algorithms in **pr2_algorithms** and run them in the PR2 realtime loop
* **pr2_joint_position_controllers**: Provides the generic joint controller that is shared by all the cartesian controllers