# SLAM

## Abstract
The goal of slam is trying to locate the flight and build the map of the environment around the flight at the same time.
## Installation
The code was tested on Ubuntu 20.04 and makesure the environment has eign3 library
### downloading opencv
step1  
download the latest source from OpenCV's GitHub Repository
```
$ sudo apt-get install git
$ git clone https://github.com/opencv/opencv.git
```
step2  
open a terminal window and navigate to the downloaded "opencv" folder. Create a new "build" folder and navigate to it.
```
$ mkdir build
$ cd build
```
source: https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html
### downloading pcl
```
$ sudo apt install libpcl-dev
```
