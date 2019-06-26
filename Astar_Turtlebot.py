import math
from sys import exit
import numpy as np

robot_l = 0.354/2
clearance = 0.5

d = robot_l + clearance
def obstacle(x, y):
    c=0
    if x<=-5.55+d or x >=5.55-d or y<=-5.05+d or y>=5.05-d:
        c=1
    elif ((x+1.6)**2 + (y+4.6)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y+2.31)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y-2.31)**2 ) <= (0.405 + d)**2 :
        c=1
    elif ((x+1.65)**2 + (y-4.6)**2 ) <= (0.405 + d)**2 :
        c=1  
    elif ((x+4.05)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2 or ((x+2.46)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2:
        c=1
    elif -4.05 - d <= x and x <= -2.45 + d and 2.45 - d <= y and y <= 4.05 + d:
        c=1
    elif 1.3 - d <= x and x <= 5.55 + d and -5.05 - d <= y and y <= -4.7 + d:       
        c=1
    elif -0.81 - d <= x and x <= 1.93 + d and -4.7 - d <= y and y <= -3.18 + d:       
        c=1
    elif 2.24 - d <= x and x <= 3.41 + d and -4.7 - d <= y and y <= -4.12 + d:    
        c=1
    elif 3.72 - d <= x and x <= 5.55 + d and -4.7 - d <= y and y <= -3.94 + d:       
        c=1
    elif -1.17 - d <= x and x <= -0.26 + d and -1.9 - d <= y and y <= -0.08 + d:      
        c=1      
    elif -0.26 - d <= x and x <= 1.57 + d and -2.4 - d <= y and y <= -1.64 + d:     
        c=1
    elif 2.29 - d <= x and x <= 3.8 + d and -2.38 - d <= y and y <= -1.21 + d:       
        c=1    
    elif 4.97 - d <= x and x <= 5.55 + d and -3.27 - d <= y and y <= -2.1 + d:       
        c=1    
    elif 4.64 - d <= x and x <= 5.55 + d and -1.42 - d <= y and y <= -0.57 + d:     
        c=1  
    elif 4.97 - d <= x and x <= 5.55 + d and -0.57 - d <= y and y <= 0.6 + d:       
        c=1  
    elif 1.89 - d <= x and x <= 5.55 + d and 1.16 - d <= y and y <= 1.92 + d:    
        c=1
    elif 2.77 - d <= x and x <= 3.63 + d and 3.22 - d <= y and y <= 5.05 + d:        
        c=1   
    elif 4.28 - d <= x and x <= 4.71 + d and 4.14 - d <= y and y <= 5.05 + d:       
        c=1
        
    return c

def start(initial):
    q=obstacle(initial[0],initial[1])
    if q ==1: 
        print("Start point inside obstacle space or not in workspace space or not a good entry for resolution")
        exit()
    else:
        pass
def end_point(initial):
    q=obstacle(initial[0],initial[1])
    if q ==1: 
        print("Final point inside obstacle space or not in workspace space or not a good entry for resolution")
        exit()
    else:
        pass

import matplotlib.pyplot as plt
x=input("Please input x value for initial node in meter")
y=input("Please input y value for initial node in meter")
theta=float(input("Please input value for initial theta in radians "))
x1=input("Please input x value for goal node in meter")
y1=input("Please input y value for goal node in meter")


initial = [float(x),float(y)]
start(initial)
final_n = [float(x1),float(y1)]
end_point(final_n)
radius = (0.038)
vel1 = 5
vel2 = 10
visited = []

dt = 0
dtime = 2
L = 0.230
current = [initial]

All_nodes = [initial]
parent = [initial]

in_cost = 0
cost = []
heuristic_cost = []
cost.append(float('inf'))
heuristic_cost.append(float('inf'))

parent_visited = []
velocity=[]
velocity_new=[]
index_heuristic_least  = 0
velocity.append([0,0])
theta_list=[]
theta_list.append(theta)


def inbetweenpoint(n, p):
    flag = 0
    for point in p:
        if (((n[0] - point[0])**2 + (n[1] - point[1])**2) - (0.1)**2 < 0):
            return True
    return False

def feasable(current, initial,ul,ur,dt):
    h2=heuristic(initial,current)
    h1 = heuristic(current, final_n)
    cost_initial = h2 +in_cost 
    costh = cost_initial + h1
    current = [ ((math.floor(current[0] * 100)) / 100.0), ((math.floor(current[1] * 100)) / 100.0) ]
    q=obstacle(current[0],current[1])
    f = inbetweenpoint(current, visited)
    if current not in visited and q == 0 and f== False :
        if current in All_nodes:
            check = All_nodes.index(current)    
            check_cost = heuristic_cost[check]
            if check_cost <= costh:
                pass
            else:
                All_nodes.pop(check)
                cost.pop(check)
                parent.pop(check)
                heuristic_cost.pop(check)
                velocity.pop(check)
                theta_list.pop(check)
                theta_list.append(dt)
                cost.append(cost_initial)
                All_nodes.append(current)
                parent.append(initial)
                heuristic_cost.append(costh)
                velocity.append([ul,ur])
        else:
            theta_list.append(dt)
            cost.append(cost_initial)
            All_nodes.append(current)
            parent.append(initial)
            heuristic_cost.append(costh)
            velocity.append([ul,ur])
    else:
        pass
        
def generate(ul, ur, initial,theta_list):
    xd = initial[0]
    yd = initial[1]
    dt = theta_list
    for i in range(0,100):
        xd = xd + (radius/2)*(ul + ur)*math.cos(dt)*dtime*(1/100)
        yd = yd + (radius/2)*(ul + ur)*math.sin(dt)*dtime*(1/100)
        dt = dt + (radius/L)*(ur-ul)*dtime*(1/100)
        q=obstacle(xd,yd)
        if q==1:
            break
            
    current = (xd, yd)
    feasable(current, initial,ul,ur,dt)
    
def heuristic(current, final_n):
    h1 = math.sqrt((current[0] - final_n[0])**2 + (current[1] - final_n[1])**2) 
    h1 = round(h1, 2)
    return h1

def child(vel1, vel2, initial,theta_list):
    one(vel1,vel1, initial,theta_list)
    two(vel2,vel2, initial,theta_list)
    three(0,vel1, initial,theta_list)
    four(0,vel2, initial,theta_list)
    five(vel1,0, initial,theta_list)
    six(vel2,0, initial,theta_list)
    seven(vel1,vel2, initial,theta_list)
    eight(vel2,vel1, initial,theta_list)
    
def one(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def two(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def three(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def four(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def five(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def six(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def seven(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
def eight(vel1,vel2, initial,theta_list):
    generate(vel1, vel2, initial,theta_list)
        

        
def goal_radius(n):
    c=0
    if ((n[0]-final_n[0])**2+(n[1]-final_n[1])**2<(0.1)**2):
        c=1
    return c

c = 0
count = 0
print ("---------------Nodes are being explored----------------") 
while c!=1:
    count = count +1
    if All_nodes==[]:
        print("Path not found for the given clearance")
        exit()
    child(vel1, vel2, All_nodes[index_heuristic_least ],theta_list[index_heuristic_least ])
    visited.append(All_nodes[index_heuristic_least ])
    parent_visited.append(parent[index_heuristic_least ])
    velocity_new.append(velocity[index_heuristic_least ])
    All_nodes.pop(index_heuristic_least )
    cost.pop(index_heuristic_least )
    parent.pop(index_heuristic_least )
    heuristic_cost.pop(index_heuristic_least )
    velocity.pop(index_heuristic_least )
    theta_list.pop(index_heuristic_least )
    if heuristic_cost != []:
        min_cost = min(heuristic_cost)
        index_heuristic_least  = heuristic_cost.index(min_cost)
        in_cost = cost[index_heuristic_least ]
    c = goal_radius(All_nodes[index_heuristic_least ])

new_goal = []
velocity_goal=[]
goal = visited[-1]
new_goal.append(goal)
velocity_goal.append(velocity_new[-1])

while True:
        check = visited.index(goal)
        goal = parent_visited[check] 
        new_goal.append(goal)
        velocity_goal.append(velocity_new[check])
        if goal == initial:
            break

def generateMap(visited, path):

    xcoor = np.linspace(-5.5,5.5,110)
    ycoor = np.linspace(-5,5,100)
    
    
    
    for i in visited:
        plt.scatter(i[0], i[1], s=1,color = 'r')
    
    for i in path:
        plt.scatter(i[0], i[1], s=1,color = 'b')
    
    for x1 in xcoor:
        for y1 in ycoor:
            if obstacle(x1, y1) == True:
                plt.scatter(x1, y1, color = 'k')
        
    plt.show()
    return 0
print ("---------------Map is being formed----------------") 

generateMap(visited,new_goal)
#print(velocity_goal[::-1])
print("------Please start the simulation in vrep and then close the Map--------")

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    time = 0
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    path_vel=velocity_goal[::-1]
    for k in path_vel:
        time = 0
        err_code1 = 1
        err_code2 = 2
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            #print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            #print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<=2):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    vrep.simxGetPingTime(clientID)
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

