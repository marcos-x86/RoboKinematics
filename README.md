# RoboKinematics

Description
-----------
RoboKinematics is a basic application for students in electronics, mechanics and mechatronics. This application allows to calculate the forward kinematics and the inverse kinematics of industrial robots with rotational joints, up to 3 degrees of freedom (RRR).

General Instructions
--------------------
A industrial robotic arm is made up of links and joints. The links are the elements that unite the joints, the latter are those that form the degrees of freedom.

Then we outline the general steps to use the application.

1. Setup Your Robot

The first step is to configure the basic properties of the robot, such as:
- The number of degrees of freedom
- The length of the links
- The limit of joints
The App shows images that help in setting these parameters.

If there is any error in the configuration of the parameters a warning message is displayed.

2. Calculate the kinematic

The second step is to enter data for the calculation of the kinematics

- In the case of the forward kinematics, rotation angles of the joints should be introduced so that the App calculates the coordinates of the end effector position.
- In the case of inverse kinematics, the coordinates where you want to position the effector must be introduced so that the App calculate the angles of joints

App includes three buttons to move the ScrollView to its initial position, to erase all the data entered values ??and return to default settings App.

Development information
-----------------------
IDE: Android Studio 1.5.1
API Level: 15. Android 4.0.3 (ICE_CREAM_SANDWICH_MR1)
JDK: 1.7.0 Update80 32bit
OS: Linux Mint 17.1 Rebecca 32bit

Contact information
-------------------
Name: Marcos Lara Torrico
Email: learner.mlt@gmail.com
