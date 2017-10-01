# README #

Imperio is Robotic Autonomy. It is the top level software for controlling the robot in competition.

## Versions  ##
1. __Turtlebot__ : The skeletal structure of the code that runs the commands on a turtlebot. Many of the methods are stub methods just to show how the code _would_ work. Most of the focus here is the architecture/design of the code. 
2. __Simulator__
3. __NDSU Physical Robot__

### What is this repository for? ###

* Autonomous software package for NDSU submission of the NASA Robotic Mining Competition
* Version : 1.0

### How do I get set up? ###

* Make sure catkin is properly installed
* Downlaod as a part of the overall src directory
* Navigate to the top of the catkin workspace
* From the terminal
  >catkin_make  
  >source devel/setup.bash    
  >rospack profile  
  

### How do I run it? ###

* From the terminal:  
   >roslaunch

* From a seperate terminal:  
   >roslaunch imperio __(preferedRobot)__.launch  
   >python src/imperio.py   

_note: this will change once it becomes it's own node_
### Contribution guidelines ###

 * For right now, please don't touch this code without a really good reason and talking to me first.

### Who do I talk to? ###

* Nicole Maguire :   
 Email : Maguirne@dukes.jmu.edu  
 [GitHub : NicoleNotUnix](https://github.com/NicoleNotUnix)  
 [BitBucket : NicoleNotUnix](https://bitbucket.org/NicoleNotUnix/)  
