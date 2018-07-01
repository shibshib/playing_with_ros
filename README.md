# Playing with ROS

This is a small 2-day project I did to play around with Robot Operating System (ROS) and pointclouds. Prior to this project I had no experience working with ROS or with pointclouds for that matter, as such it was a fun endeavour to learn how it all ties together in one small project.

In this project, a node subscribes to a pointcloud topic (say, Velodyne). It then does a rotation and applies an offset on that pointcloud, and publishes it again for another node to pick up the linearly translated and rotated pointcloud. 

## Structure
In this project, there are multiple files:
1. *cloud_manipulation.py* - is a library that contains the cloud manipulation code. 
2. *node.py* - is the node that subscribes to the pointcloud topic, applies the manipulations and republishes it. It takes in 3 commandline arguments: 
	--1. Rotation angle
	--2. Rotation axis
	--3. Offset
3. *unit_tests.py* - which unit test the cloud_manipulation.py library to make sure that everything works as it should, with maximum code coverage. 

## Operation
This project is written in Python 2.7, a future endeavour is to update it to become Python 3.6+ compatible. To run the project, simply run the script run.sh. This script grabs a bag file called test.bag that is in the same directory and plays it. Meanwhile, it runs the node.py script to subscribe to the test.bag's topic, manipulate the pointcloud data, and publish it. 

Additionally, to run the unit_tests.py and see how the unit tests perform, simply run it using python without any command line arguments, for example `python unit_tests.py`. 


Note that this project was written using only python standard libraries, and as such many functions could have been written better and more efficiently. This is known by me, but for the purposes of learning more about how it works and demonstrating the code, it remains as is for now.  
