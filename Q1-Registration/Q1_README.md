# README file for Q1 Registration
## 1.Introduction
There are some ambiguities in the requirements of the topic, so I try my best to write code to meet your requirements. In this question, I mainly write two cpp files. The first one is a service in `horn_method_service.cpp` and the second one is a library called `helper_functions.cpp`
### 1.1horn_method_service.cpp
I define my server to take paths of two frame csv files as input parameters and output the transformation of frame1 with respect to frame2. You can check `srv/HornMethod.srv` to see the details of input and output parameters. Specially, I use `geometry_msgs/Pose` as the type of the responsed pose. It doesn't look like a 4x4 transformation matrix because it combines with translation and quaternion.
### 1.2helper_functions.cpp
It seems that you want me to write a service to implement the required functions. But I think most of ROS message types don't support 4x4 matrix. Instead, they are stored as translation and rotation speratedlly. So I'm confused with the requirements. But I still write the functions and make them as libraries in case you want test them. Also in `CMakeLists.txt`, you can see I have already add a new library named `helper_functions` and link it to `horn_method_service`.
## 2.Try it
- Install Eigen
```
apt install libeigen3-dev
```
- Revise the path of Eigen in `CMakeLists.txt`
Because the original Eigen package doesn't provide the `.cmake` file. So you can't use `find_package`to get it. And since the Eigen are just header files, the simple way is to add your Eigen path into `include_directories`.
- Build all files
```
catkin_make
```
- Call the service
The service name is `/horn_method`. You can directly use command in the terminal to call the service:
```
rosservice call /horn_method <PATH/TO/FRAME1/CSV> <PATH/TO/FRAME2/CSV>
```
Then, you can get the output directly from the terminal like the following:
```
pose: 
  position: 
    x: 590.8288830249007
    y: 1480.8492241360036
    z: -116.62996100512946
  orientation: 
    x: 0.018936008635992378
    y: 0.013759860075574665
    z: -0.7107933897989772
    w: 0.703011273626332
```
## 3.About the bonus
I noticed that you mentioned writing a subscribe callback, but it seems like you're actually working with a service instead of a topic. However, if you do need to store information outside of the callback function, you can define your callback function within a class. This way, you can store your information in member variables and access them outside of the callback function.

By organizing your code in this way, you can ensure that the information you need to store persists even after the callback function has finished executing. This can be particularly useful if you need to use this information in other parts of your program.