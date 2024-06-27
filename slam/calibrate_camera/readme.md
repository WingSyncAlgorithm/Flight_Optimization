# calibrate_camera
## Abstract
Trying to calibrate pictures captured by stereo camera and build some depth pictures by using opencv library.
## USAGE
### using cmake
step1  
make a build folder
```
$ mkdir build
$ cd build
```
step2  
build the project by
```
$ cmake ..
```
step3  
make the cpp files by
```
$ make
```
step4  
execute by
```
$ ./ filename
```
### detail of each programm
1.cali.cpp  
  finding intrinsic parameters of camera  
2.rectify.cpp  
  rectify the pictures captured by camera
3.match.cpp
  build depth picture from rectified pictures
