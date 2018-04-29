# README #

Imperio is Robotic Autonomy. It is the top level software for controlling the robot in competition.

### What is this repository for? ###

* Autonomous software package for NDSU submission of the NASA Robotic Mining Competition
* Version : 3.0

### How do I get set up? ###

* Make sure catkin is properly installed
* Download as a part of the overall src directory
* Get the repository from github
  >git checkout autonomy
* Navigate to the top of the catkin workspace
* From the terminal
  >catkin_make  
  >source devel/setup.bash    
  >rospack profile  
  
### How do I run it? ###

* From the terminal:  
   >roscore

* From a seperate terminal:  
   >roslaunch imperio imperio.launch --screen
   
   this will launch the imperio node, the imperio visualization tool, and the competition timer.

### How do I test it? ###

* Install the python testing frameworks if you don't already have them
   >pip install pytest
   
   >pip install pytest-cov

* To run the tests (Can be done from any directory containing imperio/scripts)
   >roslaunch imperio unit_testing.launch

   >pytest

* Run the code coverage (Assuming you're at the top of the workspace)
   >pytest --cov=src/imperio/scripts
* More in-depth code coverage information
   >coverage report -m
* Extremely in-depth code coverage information (recommended) can use any browser
   >coverage html

   >firefox htmlcov/index.html

### Contribution guidelines ###

 * Just Don't Touch It

### Who do I talk to? ###

* Nicole Maguire :   
 Email : Maguirne@dukes.jmu.edu  
 [GitHub : NicoleNotUnix](https://github.com/NicoleNotUnix)  


