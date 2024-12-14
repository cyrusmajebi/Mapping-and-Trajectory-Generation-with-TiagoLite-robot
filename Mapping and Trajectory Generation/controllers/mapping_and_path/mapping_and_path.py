from controller import Robot, Supervisor
from matplotlib import pyplot as plt
from os.path import exists 
from scipy import signal

import numpy as np
import sys
import os
import ast

SCRIPTS_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..', 'Scripts'))
#print(SCRIPTS_FOLDER)
sys.path.append(SCRIPTS_FOLDER)
from rrt import rrt

# Create the Robot instance.
robot = Supervisor()


# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# timestep = 32


# Get the instance of a device of the robot
motor_left = robot.getDevice('wheel_left_joint')
motor_right = robot.getDevice('wheel_right_joint')


motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
# Insert a getDevice-like function in order to get the
# instance of a device of the TIAGo robot. Enable Lidar, GPS, Compass
# and display.

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)



MAX_SPEED = 10.1 # Maximum speed of the TIAGo motors.

x_w = 0.0 # Inital robot x-coordinate in world frame where 
            # the robot is placed slightly above 
y_w = 0.0 # Inital robot y-coordinate in world frame.
             
back_to_start = False # Boolean variable used to check if the 
                      # robot has returned to the start points. 


# Data structure to hold the angles values for the lidar ranges.
# The value of 4.19 is gotten from the Hokuyo lidar's fieldOfView 
# field in the scene tree. 667 is its horizontal resolution.
angles = np.linspace(4.19/2,-4.19/2,667) 
                         

# A function that maps world coordinates to map coordinates
def world2map(x_w,y_w):
    
   x_w_origin = -2.16
   y_w_origin = 1.78
   p_x_origin = 0.0
   p_y_origin = 0.0
   
   x_w_end = 2.42
   y_w_end = -4.07
   p_x_end = 199.0
   p_y_end = 299.0
   
   px = int(( ((x_w - x_w_origin) * (p_x_end - p_x_origin)) / \
         (x_w_end - x_w_origin)) + p_x_origin)
         
   py = int(( ((y_w - y_w_origin) * (p_y_end - p_y_origin)) / \
         (y_w_end - y_w_origin)) + p_y_origin)
   
   px = min(px, 199)
   py = min(py, 299)
   px = max(px, 0)
   py = max(py, 0)
   
   return [px,py]

# A function that maps map coordinates to world coordinates
def map2world(p_x,p_y):
    
    
   p_x_origin = 0.0
   p_y_origin = 0.0
   x_w_origin = -2.16
   y_w_origin = 1.78
   
   p_x_end = 199.0
   p_y_end = 299.0
   x_w_end = 2.42
   y_w_end = -4.07

   
   x_w = ( ((p_x - p_x_origin) * (x_w_end - x_w_origin)) / \
         (p_x_end - p_x_origin)) + x_w_origin
         
   y_w = ( ((p_y - p_y_origin) * (y_w_end - y_w_origin)) / \
         (p_y_end - p_y_origin)) + y_w_origin
   
   
   return [x_w,y_w]

# A probabilistic map data structure 
#map = np.zeros((200,300))
cmap = False




marker = robot.getFromDef("marker").getField("translation")


# Array containing the waypoints for the trajectory.
WP = [(0.61,-0.61), (0.61,-2.91), (-0.72,-3.36), 
      (-1.83,-2.66), (-1.72, -1.61), (-1.56,-0.32), 
      (-0.55, 0.55), (-1.82, -0.06), 
      (-1.72, -1.61), (-1.83,-2.66), (-0.72,-3.36),
      (0.61,-2.91), (0.61,-0.61), (0,0)]
      
# Starting index
index = 0


path_to_file = '''C:/Users/Cyrus/Desktop/Online courses/Introduction to Robotics with Webots 
/mapping_and_traj_gen/controllers/mapping_and_path/cspace.npy'''
map = np.load(path_to_file)
file_exists = exists(path_to_file)

if file_exists:
    print("Reading map")
    print(map.shape)
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            # Draw map on display
            if map[i][j] == True:
                # mx, my = map[i], map[j]
                # print(mx, my)
                display.setColor(0xFFFFFF)
                display.drawPixel(i,j)
                
    q_start = (0.0, 0.0)
    q_goal = (-0.72, -3.36)
    q_start_world = tuple(world2map(q_start[0],q_start[1]))
    q_goal_world = tuple(world2map(q_goal[0],q_goal[1]))
    print(f"q_start_world => {q_start_world}")
    print(f"q_goal_world => {q_goal_world}")
   
    rrt(map, q_start_world, q_goal_world)
    path_waypoints = None
    with open('path.txt', mode='r') as file:
        content = file.readline()
        if content:
            try:
                path_waypoints = ast.literal_eval(content)
                if isinstance(path_waypoints, list):
                    print(f"path_waypoints => {path_waypoints}")  
                else:
                    print("File does not contain a valid list of tuples.")
            except (ValueError, SyntaxError):
                print("File contains invalid Python expression.")
            print(type(path_waypoints))
            
            # Draw shortest path.
            for p in path_waypoints:
                #print("plotting shortest path on display")
                (px, py) = map2world(p[0],p[1])
                (px, py) = (round(px, 2), round(py, 2))
                #print(p)
                #print((px, py))
                display.setColor(0x00FFFF)
                display.drawPixel(p[0],p[1])
            
            file.close()
        else:
            print("File is empty.")
            
    
    
    #print(map2world(62, 162))
    
else:
    print("No map. Mapping environment")

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
    
        # Get position and heading for robot from GPS and compass.
        x_w = gps.getValues()[0] 
        y_w = gps.getValues()[1] 
        theta=np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
        
        # Set the position of the marker and compute rho and alpha.
        marker.setSFVec3f([*WP[index], 0.02])
        rho = np.sqrt((x_w-WP[index][0])**2 + (y_w-WP[index][1])**2)
        alpha = np.arctan2(WP[index][1]-y_w, WP[index][0]-x_w)-theta
        
      
        # Fixes the non-linearity associated with the np.arctan2 function.
        if (alpha > np.pi):
            alpha = alpha - 2*np.pi  
        if (alpha < -np.pi):
            alpha = alpha + 2*np.pi
    
    
        # Check if the robot is close enough to the marker
        # Increment the index if so.
        if (rho<0.35):
            index += 1
          
        
        # Read and preprocess range images.
        w_T_r = np.array([[np.cos(theta),-np.sin(theta), x_w],
                          [np.sin(theta),np.cos(theta), y_w],
                          [0,0,1]])            
        ranges = np.array(lidar.getRangeImage())
        ranges[ranges == np.inf] = 100
        
        X_r = np.array([ranges*np.cos(angles), ranges*np.sin(angles),
                        np.ones(len(angles))])
                        
        X_r[0,:] += 0.2 # Add a 0.2m offset to all the x-values of X_r
                        # because the lidar unit is offset from the robot's
                        # origin by 0.2m/20cm.
                        
        X_r = X_r[:,80:587] # Drop the first and last 80 readings of the X_r array
                            # because Tiago's Lidar is inside a box that shields 
                            # the first and last 80 readings.
                            
        D = np.dot(w_T_r, X_r) # Transform range image into world coordinates
      
      
      
        # Draw odometry.
        px, py = world2map(x_w,y_w)
        display.setColor(0xFF0000)
        display.drawPixel(px,py)
    
        
     
        # Store values on map.
        for d in D.transpose():
            mx, my = world2map(d[0], d[1])
            map[mx, my] += 0.01
            
            if (map[mx, my] > 1):
                map[mx, my] = 1
                
            v = int(map[mx,my]*255)
            color = ( (v*256**2) + (v*256) + (v) )
            
            display.setColor(int(color))
            display.drawPixel(mx,my)
            
        
        # Process sensor data and calculate wheel speeds with 
        # a proportional controller.
        p1 = 5
        p2 = 4
     
        leftSpeed = -alpha*p1 + rho*p2
        rightSpeed = alpha*p1 + rho*p2
        
        # Reduce calculated speeds by a factor to prevent unstable 
        # motions.
        leftSpeed *= 0.6
        rightSpeed *= 0.6
        
        # Cap will speeds between the robots max speed.
        leftSpeed = max(min(leftSpeed, MAX_SPEED),-MAX_SPEED)
        rightSpeed = max(min(rightSpeed, MAX_SPEED),-MAX_SPEED) 
        
      
        # Set Actuator values.
        motor_left.setVelocity(leftSpeed)
        motor_right.setVelocity(rightSpeed)
        
        
        # If the last waypoint has been reached, set the back_to_start
        # boolean to True
        if index == len(WP):
             back_to_start = True
        
        
        # Draw the configuration space map with a kernel size of 27.
        if (cmap==False and back_to_start==True):
            print("Arrived")
            cmap=True
            kernel= np.ones((26,26)) 
            cmap = signal.convolve2d(map, kernel, mode='same')
            cspace = cmap>0.9
            
            np.save('cspace',cspace)
           
            plt.imshow(cspace)
            plt.show()
        
        
        #Check if the robot has returned to the start point, 
        # stop the simulation of it has. 
        if back_to_start:
            motor_left.setVelocity(0.0)
            motor_right.setVelocity(0.0)
            sys.exit(0)
            
        
        pass

# Enter here exit cleanup code.
