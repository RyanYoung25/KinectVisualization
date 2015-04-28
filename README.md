Visualization for the kinect using OpenNi
=========================================

For explorer Sunday we needed a visualizer for the kinect sensor. We wanted to show kids what the kinect saw 
skeleton wise and show them that we could calculate the angles and bend of some of the joints. This was used 
in conjuction with an oral presentation describing computer vision and how the kinect sensor works. 

The simulation is displayed using matplotlib. The window size is fixed in code and can not be resized on my machine. 
I'm using a very old version of Matplotlib, 1.1.1rc, because a few other things on my computer will break if I update it. 


Installation
============

The only thing that needs to "installed" is that this package needs to be added to your `ROS_PACKAGE_PATH` in your .bashrc
The install script does this for you automatically, just run ./install

Uninstallation
==============

To uninstall this remove the line that was added to your .bashrc and delete this repo. 

Running
=======

To run, run the script VisualizeKinect.py


Dependencies
============

This project will require a few dependencies:

- OpenNi ROS package
- OpenNi_tracker ros package
- Matplotlib 
