ENPM661-P3

A* algorithm on Turtlebot-2 , Simulation on V-rep

####### Libraries to be imported ###############################################################
	- import math
	- import sys
	- import matplotlib.pyplot as plt
	- import numpy as np
	- import time
       

###### Files need to be included in the same folder to have established Remote API bindings #######
	- make sure "vrep.py" and "vrepConst.py" are also present in the same folder


##### Parameters defined in the program ##########################################################
	- clearence - 50 cm
	- clearence for the obstacle space using half planes ((diameter_of_robot/2)+clearence) = 67 cm
	- RPM 1 = 5 rad/sec
	- RPM 2 = 10 rad/sec
	- Length between wheels = 23 cm
	- dt = 2 sec
	- The origin is at center of the scene
	- x axis ranges from (-5.55,5.55) and y axis ranges from (-5.05,5.05)

#######Steps to run the code#########################################################################
	- Open V-REP and place the turtle-bot at the start location with initial orientation and initiate the simulation
	- Open the terminal in the saved folder location and run the file "Astar_Turtlebot.py" 
	- Enter the exact location and orientation of the robot in the terminal, as it was placed in the V-REP
	- Scatter plot shows the graphicaly node exploration and the path and the final path
	- Robot simulation then starts in V-REP

######## Map Details ###############################################################################
	- Black colour denotes obstacles
	- Red color denotes explored nodes
	- Blue color denotes the path 

########There are 3 video samples along with the code with simulation ################################
	- "[0,4]to[-4,-4].avi" with initial cordinate(0,4) and goal at (-4,-4) with initial orientation as 0 radians
	- "[-4,-4]to[1,1].avi" with initial cordinate(-4,-4) and goal at (1,1) with initial orientation as 0 radians	
	- "[1,0]to[-4,0].avi" with initial cordinate(1,0) and goal at (-4,0) with initial orientation as 0 radians



