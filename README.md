dnepr
=====

# Building dnepr_quadro_control
## Prepare the system

* Get Raspbian image here: http://goo.gl/FjGKb
* Prepare an SD using http://sourceforge.net/projects/win32diskimager
* Start the Pi.
* Use $ raspi-config to expand the SD partition.
* Now you can use RPi via SSH.

## Set up apt for ROS

* $ sudo sh -c 'echo "deb http://64.91.227.57/repos/rospbian wheezy main" > /etc/apt/sources.list.d/rospbian.list'
* $ wget http://64.91.227.57/repos/rospbian.key -O - | sudo apt-key add -
* $ sudo apt-get update

## Working folder

* Git clone dnepr_quadro_control to /home/pi/projects/dnepr/RaspberryPi/dnepr_quadro_control
* $ export ROS_PACKAGE_PATH=/home/pi/projects/dnepr/RaspberryPi:$ROS_PACKAGE_PATH
* $ cd dnepr_quadro_control
* $ source setup.bash
* $ rosdep update (<i>? maybe don't need this<i>)
* $ sudo apt-get install ros-groovy-openni-camera
* $ sudo apt-get install ros-groovy-opencv2

# "Failed to download target platform data for gbpdistro" error at "rosdep update" step
(http://answers.ros.org/question/9201/how-do-i-install-a-missing-ros-package/)
* sudo pip uninstall rosdep
* sudo pip install rosdep

