# CAM-RobSoftDev-2022



## SETUP

* You should use Ubuntu 20.04 Focal Fossa to run ROS: https://releases.ubuntu.com/focal/
* If you have a dedicated hard drive you can create a bootable usb drive with Rufus: https://rufus.ie/en/
* Or you can download Docker: https://docs.docker.com/get-docker/
* Or you can download VMWare: https://www.vmware.com/au/products/workstation-player/workstation-player-evaluation.html
* Install ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu

#
# QUESTIONS

## Question 1: Create a ROS Library that performs registration of two objects wrt to each other

In Robotics, pose estimation is an important problem. For e.g. When we are dealing with robots handling objects in its workspace, it becomes important to localize these objects with respect to a base frame of reference (In most of the cases the base will be Robot's base frame also known as world). There are several methods that are used to perform pose estimation or localization. One of the widely used methods is the Horn's method that computes the pose of one entity with respect to the other. Here are some links for you to get a better understanding of this:https://roboticsknowledgebase.com/wiki/math/registration-techniques/, http://nghiaho.com/?page_id=671.

In this question we would like you to create a ROS library that uses the Horn's method to compute the Transformation/Pose of Frame2 with respect to Frame1

* General Requirements and Design of your library should look as follows:
1. The input to your code will be two files: Frame1.csv and Frame2.csv. These files can be found in the data folder of Question 1. The files consist of XYZ coordinates of four points with respect to frame1 and frame2 respectively. Your library should have a parsing function that reads these files and stores them in appropriate data structures
2. Your library should have the following functionalities:
    1. A function as a ROS Service that is the implementation of Horn's Method and computes the homogeneous transformation (4x4 Matrix) between Frame1 and Frame2 (Note: Here Frame1 is the base frame). Make sure to use appropriate ros msgs for defining Transforms (Hint: Look at geometry_msgs) in your service file description
    1. Helper functions that convert: a rotation matrix to quaternion, a rotation matrix to Euler angles (ZYX), and vice-versa.
    1. Helper functions that compute the inverse transformation. For e.g. if we give a Transformation matrix that is the pose of Frame2 with reference to Frame1, then this function computes the pose of Frame1 with respect to Frame2
    1. Bonus: Make subscriber callbacks for the functions in part b and c and store the return types as a class variables in your node definition. Since subscriber callbacks do not have a return type, you need to store the converted values in a variable that can be accessed in runtime.
3. You should only use C++ to make this library. The external library that you should be using is Eigen (https://eigen.tuxfamily.org). You can use any in-built function from the aforementioned libraries to program the described functionalities.
4. The functions/methods in your library can have a return type of any standard c++ datatypes in the STL library or any Eigen datatypes. For your service definitions you can use any open source msgs from ROS (std_msgs, geometry_msgs, sensor_msgs, etc. )
5. Some resources that you might find useful: https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html, https://www.andre-gaschler.com/rotationconverter/
6. Some exhaustive resources: https://www.seas.upenn.edu/~meam620/notes/RigidBodyMotion3.pdf, https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf, https://www.cs.rpi.edu/~trink/Courses/RobotManipulation/lectures/lecture6.pdf
![Expected Transformation Output for the Given Data: ](/Images/Q1_Image1.PNG)

#
## Question 2: PointCloud Registration

The previous method for registration requires a one-to-one correspondence between the two frames (e.g. The 4 points given in Q1 are the same four points represented in different frame of references). Now this might not be practical in several real world scenarios that we face while doing research. For e.g. when you collect a PointCloud using a 3D Camera, the pointcloud is wrt to the camera but you might want to represent this in another frame of reference. This question will help you understand a method known as Iterative Closest Point Algorithm (ICP) for finding the transformation between two objects when there is no one to one correspondence available.
* General requirements and Design of your Package:
1. You will be given two files: First File is a PointCloud of a scanned object (PointCloud.ply) and the second one is a CAD file of the same part (CAD.stl). These files can be found in data folder of Q2.
2. You will generate a ROS package that will do the following:
    1. Parses these files and stores them in appropriate data structures
    1. Apply any preprocessing if necessary to clean the PointCloud, you can use the following tool to visualize the PointCloud before you begin programming (https://www.meshlab.net/)
    1. Create methods or functions that compute the transformation of the PointCloud with respect to the CAD's frame of reference
    1. Align the PointCloud with the CAD using the ICP algorithm
    1. ICP in PCL library requires an initial guess. You can use a random guess or a smart way of solving this would be to use your code from Q1 by locating 4 points in the Pointcloud that match 4 points on the mesh. For example if you had a CAD of a cube and a pointcloud of the same cube, you could match 4 corners with each other. You can then compute the transformation using the Horn's method and use this as an initial guess.
3. You have the freedom of using C++/Python for this question but you need to use PCL library.
4. You can use numpy or Eigen if required. You can use any in-built functions in C++ STL, Python-Numpy/Math, and PCL.
5. Once you have the aligned PointCloud you should publish it to a topic name "/Question2/AlignedPointCloud" using appropriate ROS Msg (E.g. PointCloud2 msg)
6. Make a results folder in your Question2/src/results. You will take an image of the aligned PointCloud visualized in RVIZ. The RViZ window should consist of the CAD as well as the PointCloud. A correctly aligned result should have both of them superimposed over each other.
![The Unaligned PointCloud and the CAD looks as follows: ](/Images/Q2_Image1.png)

#
## Question 3: Collision Detector

This Question will test your Algorithms Knowledge. You can use Python with numpy or C++ with STL, EIGEN for Linear Algebra Functionality.
In Robotics, collision detection is a feature that has very high complexity.
In this question, you will create a simple collision detector. Write out your answers to the subquestions and save them in the Q3-Collision Detector readme.md
### The Questions are:
* Assumptions: A Box is a geometric entity that can either be a cube or rectangular parallelopiped. The faces of the box that you are working with are inline with the xy-xz-yz planes (i.e. orientation of the box is fixed). You are given the following classes.

```c
Class Point {
double x;
double y;
double z;
Point(x,y,z);
}

Class Box {
string name;
Point a; //a is one of the corner points that defines the diagonal of the box (Here diagonal is not the face diagonal)
Point b; //b is the corner point opposite to a that defines the diagonal of the box
Box(Point a, Point b); 
}
```

1. You are working in a 3d coordinate system.
    1) Write a function that takes in a Box and a Point, and returns whether or not the point lies within the Box.
    2) Write a function that takes in two Boxes and returns whether the Boxes intersect.
    3) Write a function that takes in a Box and a point, and returns whether a point is on the faces of the Box.
    4) Input: a large set of boxes and a small set of points, write an optimized function to return 2 arrays. 
        1) Each intersecting box to box pair 
        1) An array of all boxes that each point is found within

Be prepared to discuss the following questions:
* What is the running time complexity of your algorithm?
* What is the memory complexity of your algorithm?
* Describe how your algorithm could be parallelized with a GPU (no coding needed)


2. Algorithm coding test 
* Part 1:
    * Use RVIZ visual tools to visualize the collision between two 3D boxes or a point(approximated as a unit sphere) and a box
Show test cases for your code where two boxes are in collision or a point is in collision with the box (inside or on one of the faces) as well as showing a test case in which the boxes are not in collision.
Attach the images of your RViz visualization in the readme/pdf submission. Make sure to submit your code for RViz too
Refer to this link: https://github.com/PickNikRobotics/rviz_visual_tools

* Part 2:
    * You are working in a 2d coordinate system. You are given the following classes:
```c
Class Point{
double x;
double y;
Point(x,y);
}

Class Rectangle{
// a, b refer to opposing points of the rectangle like in the diagram
Point a;
Point b;
Rectangle(Point a, Point b);
}
```

   1. Write a function that takes in a rectangle and a point, and returns whether or not the point lies within the rectangle
   1. Write a function that takes in two rectangles and returns whether or not the rectangles overlap.

* Part 3: The classes are modified to store integer coordinates:
```c
Class Point{
int x;
int y;
Point(x,y);
}

Class Rectangle{
// a, b refer to opposing points of the rectangle
Point a;
Point b;
Rectangle(Point a, Point b);
}
```

* You are given a 2d array of doubles.
   * Write a function that returns the sum of all doubles within a rectangle in the 2d array.
```c
Double rectangleSum(double[][] values, Rectangle rect){
}
```
Be prepared to discuss the following questions:
* What is the running time complexity of your algorithm?
* What is the memory complexity of your algorithm?  

#
## Question 4: Vision

Using Python3 and OpenCV2, find the pixel values of the corners of the blue sheet for a given input image. Mark these corners in an output image. 
Two example images are given, your code will be run on a 3rd image of similar lighting conditions but different sheet orientation. The sheet will always be within the outer edges of the 4 ArUco markers on the right. It can be useful to find the locations of the ArUco markers and run your image processing filters/ algorithms on a cropped image with markers hidden.

#
## Question 5: Timeframe Alignment

When there is data collection that is used for model training, there are always multiple sources of sensor data that must be processed. Each of the raw data streams will have timestamps that are not synchronized, and the data must be processed so that the timestamps are aligned. 

In this problem, you are dealing with a robot attempting to insert a screw into a panel. You are given images from the robot end effector camera showing the screwdriving tool and the part. You are also given robot data dictating the end effector position [x,y,z, A, B, C], the force/torque at the end effector [Fx, Fy, Fz, Tx, Ty, Tz], and the end effector Velocity [Vx, Vy, Vz, Wx, Wy, Wz]. The Z-axis points in the same direction as the screw, and the x-axis points up. We are only interested in data points with timestamps after the screw makes contact. The goal is to code an algorithm that time-aligns the data and removes any data points that occur before the screw makes contact. The processed data should be outputted into a processed_robot_data.csv and a folder corresponding_processed_images. 

Hint: The camera is 50 ms ahead of the robot. The robot end-effector force data is one example of data points that may help detect when the robot contacts the part with the screw.

* Link to the dataset: https://drive.google.com/file/d/1DB2d7FD9iSzPq87nXCvcjLUZvZjxqG-m/view?usp=sharing
* Please use your @usc.edu email to access the raw dataset and store this in Q5-Timeframe Alignment/dataset (if you cannot access the link contact: manyar@usc.edu)

# Good luck!
