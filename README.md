dnepr
=====

# Building dnepr_quadro_control
* Get Raspbian image here: http://goo.gl/FjGKb
* Prepare an SD using http://sourceforge.net/projects/win32diskimager
* Start the Pi.
* Use $ raspi-config to expand the SD partition.
* Git clone dnepr_quadro_control to /home/pi/projects/dnepr/RaspberryPi/dnepr_quadro_control
* $ export ROS_PACKAGE_PATH=/home/pi/projects/dnepr/RaspberryPi:$ROS_PACKAGE_PATH
* $ cd dnepr_quadro_control
* $ source setup.bash
