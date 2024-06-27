# pointcloud_process
## Abstract
Trying to build a point cloud map and lighten it with octree algorithm.
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
1. joinMap.cpp  
   - given depth pictures, color pictures, and camera postion to build a point cloud map  
2. pcdviewer.cpp  
   - view the .pcd file 
3. tooctree.cpp  
   - lighten point cloud map with octree algorithm.
