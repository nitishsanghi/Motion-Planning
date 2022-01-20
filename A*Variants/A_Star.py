#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import threading
from numpy import *
from Queue import PriorityQueue

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class PlotSpinner(threading.Thread):
    def __init__(self,handle):
        threading.Thread.__init__(self)
        self.starttime = time.time()
        self.handle=handle
        self.ok = True
    def run(self):
        while self.ok:
            #self.handle.SetTransform(matrixFromAxisAngle([0,mod(time.time()-self.starttime,2*pi),0]))
            self.handle.SetShow(True)
            #time.sleep(0.01)

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def ConvertPathToTrajectory(robot,path=[]):
    if not path:
	return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

# Nodes coordinates
def xytheta_nodes_coordinates(x_start_node, x_end_node, y_start_node, y_end_node, theta_start_node, theta_end_node):
	x_nodes = [x_start_node]
	y_nodes = [y_start_node]
	theta_nodes = [round(theta_start_node,2)]

	while(x_nodes[-1]<=x_end_node):
		x_temp = round(x_nodes[-1]+step_x,2)
		if x_temp <= x_end_node:
			x_nodes.append(x_temp)
		else:
			break
	
	while(y_nodes[-1]<=y_end_node):
		y_temp = round(y_nodes[-1]+step_y,2)
		if y_temp <= y_end_node:
			y_nodes.append(y_temp)
		else:
			break
	
	while(theta_nodes[-1]<=theta_end_node):
		theta_temp = round(theta_nodes[-1]+step_theta,2)
		if theta_temp <= theta_end_node:
			theta_nodes.append(theta_temp)
		else:
			break
	return x_nodes, y_nodes, theta_nodes

# Grid 3D list created in loop
def creategrid3d(x_nodes, y_nodes, theta_nodes):
	xytheta_grid = []
	for theta_node in theta_nodes:
		xy_grid = []
		for x_node in x_nodes:
			temp_grid_column = []
			for y_node in y_nodes:
				temp_grid_column.append((x_node, y_node, theta_node))
			xy_grid.append(temp_grid_column)
		xytheta_grid.append(xy_grid)
	return xytheta_grid

class Node:
	def __init__(self,parent,location,value):
		self.parent = parent
		self.location = location
		self.value = value
		self.h = 0.0
		self.g = 0.0
		self.fcost = 0.0
		
	def f(self):
		self.fcost = self.g + self.h

	def costOfStep(self, node, metric):
		dx = abs(self.value[0] - node.value[0])
		dy = abs(self.value[1] - node.value[1])
		dtheta = abs(node.value[2] - goalconfig[2])
		dtheta = min(dtheta, 2*pi - dtheta)*.1
		if metric == 0:
			return dx + dy + dtheta
		if metric == 1:
			return sqrt(dx*dx + dy*dy + dtheta)

def convertGridtoNode(xytheta_grid): # Convert grid to nodes
	for i in range(len(xytheta_grid)):
		for j in range(len(xytheta_grid[0])):
			for k in range(len(xytheta_grid[0][0])):
				xytheta_grid[i][j][k] = Node(None,(i,j,k) ,xytheta_grid[i][j][k]) 
		
def findgoalcoord(xytheta_grid, goalconfig): #Given xytheta_grid finding coordinates of goal config
	x = goalconfig[0]
	y = goalconfig[1]
	theta = round(goalconfig[2],2)
	done = False
	for theta_coord in range(len(xytheta_grid)):
		for x_coord in range(len(xytheta_grid[0])):
			for y_coord in range(len(xytheta_grid[0][0])):
				if (xytheta_grid[theta_coord][x_coord][y_coord].value == [x, y, theta]):
					done = True
					break
			if done:
				break
		if done:
			break
	return theta_coord, x_coord, y_coord

def nodecollisioncheck(node): # Check if node leads to collision of robot
	with env:
		robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
		robot.SetActiveDOFValues(node.value)
		return env.CheckCollision(robot)

def findchildren(node, movements, xytheta_grid): #Find potential children for 4c
	#print "finding children"
	theta_lim = len(xytheta_grid) - 1
	x_lim = len(xytheta_grid[0]) - 1
	y_lim = len(xytheta_grid[0][0]) - 1
	children = []
		#print node.value
	for movement in movements:
		temp_theta = node.location[0] + movement[0]
		temp_x = node.location[1] + movement[1]
		temp_y = node.location[2] + movement[2]
		if temp_theta < 0 or temp_x < 0 or temp_y < 0:
			#print "Out of bounds"
			continue
		if temp_theta >  theta_lim or temp_x > x_lim or temp_y > y_lim:
			#print "Out of bounds"
			continue
			"""if nodecollisioncheck(xytheta_grid[temp_theta][temp_x][temp_y]):
				#print "Collision"
				continue"""
		children.append(xytheta_grid[temp_theta][temp_x][temp_y])
		#print "Valid child found"
	return children 

def heuristic(node, goalconfig, metric):
	dx = abs(node.value[0] - goalconfig[0])
	dy = abs(node.value[1] - goalconfig[1])
	dtheta = abs(node.value[2] - goalconfig[2])
	dtheta = min(dtheta, 2*pi - dtheta)*.1
 	if metric == 0:
		node.h =  dx + dy + dtheta
	if metric == 1:
		node.h =  sqrt(dx*dx + dy*dy + dtheta)

def pathfound(xytheta_grid, current):
	path = []
	while current.parent:
		path.append(current)
		current = current.parent
	path.append(current)
	return path
     
def pathfinder(goalconfig, xytheta_grid, movements, metric):
		start_time = time.clock()
		open_list = set()
		closed_list = set()
		start = xytheta_grid[2][0][0] #Start coordinates
		heuristic(start, goalconfig, metric)
		current = start
		open_list.add(current)
		while open_list:
			#current = min(open_list, key=lambda o:o.fcost)
			open_queue = PriorityQueue()
			for node in open_list:
				open_queue.put((node.fcost, node))
			current = open_queue.get_nowait()
			current = current[1]
			if current.value == (goalconfig[0], goalconfig[1], round(goalconfig[2],2)):
				print "Goal Found"
				current.f()
				#print current.g
				#print current.fcost
				#print time.clock() - start_time
				return pathfound(xytheta_grid, current)

			open_list.remove(current)
			closed_list.add(current) 

			children = findchildren(current, movements, xytheta_grid)
			for child in children:
				if child in closed_list:
					continue
				if child in open_list:
					temp_g = current.g + child.costOfStep(current, metric)
					if child.g > temp_g and not nodecollisioncheck(child):
						child.g = temp_g
						child.f()
						child.parent = current
				else:
					if nodecollisioncheck(child):
						child.g = 10000
						child.f()
						collision_config.add(child)
					else:
						collision_free_config.add(child)
						child.g = current.g + child.costOfStep(current, metric)
						heuristic(child, goalconfig, metric)
						child.f()
						child.parent = current
						open_list.add(child)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot)

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        goalconfig = [2.6,-1.3,-pi/2]
        
	start = time.clock()
        #### YOUR CODE HERE ####
	### Grid Creation
	# Start and end nodes
	x_start_node, y_start_node, theta_start_node = -3.4, -1.4, -pi
	x_end_node, y_end_node, theta_end_node = 3.4, 1.4, pi

	# Start node coordinate
	theta_start_coord = 2

	# Step Size
	step_x, step_y, step_theta = .05, .05, pi/theta_start_coord 
	
	x_nodes, y_nodes, theta_nodes = xytheta_nodes_coordinates(x_start_node, x_end_node, y_start_node, y_end_node, theta_start_node, theta_end_node) #Finding node coordinates
			
	xytheta_grid = creategrid3d(x_nodes, y_nodes, theta_nodes) # Creating grid

	# Movement steps for 4c and 8c
	movements_4c = [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]]
	movements_8c = []
	for x in [-1,0,1]:
		for y in [-1,0,1]:
			for theta in [-1,0,1]:
				movements_8c.append([theta, x, y])
	movements_8c.remove([0, 0, 0])

    #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
	convertGridtoNode(xytheta_grid)

	x_nodes_num = len(x_nodes)
	y_nodes_num = len(y_nodes)
	theta_nodes_num = len(theta_nodes)

	collision_free_config = set()
	collision_config = set()

	time_axis = []
	cost_axis = []
	start_time = 0

	metric = 1# 0 for manhattan, 1 for euclidean
	movements = movements_8c # 8 connected = movements_8c, 4 connected = movements_4c

	path_node = pathfinder(goalconfig, xytheta_grid, movements, metric)
        #### Draw the X and Y components of the configurations explored by your algorithm
	# Printing nodes
	spinner = None
	try:
		handles = []
		angles = range(0,350,10)
		xy_coord = []
		for x_node in x_nodes:
			for y_node in y_nodes:
				xy_coord.append([x_node, y_node])
		for node in xy_coord:
			handles.append(env.plot3(points=array((node[0],node[1],.05)), pointsize=2.0, colors=array(((1,1,1)))))
		for node in collision_config:
			handles.append(env.plot3(points=array((node.value[0],node.value[1],.05)), pointsize=3.0, colors=array(((1,0,0)))))
		for node in collision_free_config:
			handles.append(env.plot3(points=array((node.value[0],node.value[1],.05)), pointsize=3.0, colors=array(((0,0,1)))))
		for i in range(1,len(path_node)):
			point1 = (path_node[i-1].value[0], path_node[i-1].value[1], 0.05)
			point2 = (path_node[i].value[0], path_node[i].value[1], 0.05)
			handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((0,0,0)))))
		spinner = PlotSpinner(handles[-1])
		spinner.start()
		raw_input('Enter any key to quit. ')
		handles = None
	finally:
		if spinner is not None:
			spinner.ok = False

 	path = [] #put your final path in this variable
	if path_node:
		for node in path_node:
			path.append(node.value)
	else:
		print "No Solution Found"
	path.reverse()

        #### END OF YOUR CODE ###
	end = time.clock()
	print "Time: ", end - start

    # Now that you have computed a path, convert it to an openrave trajectory 
	traj = ConvertPathToTrajectory(robot, path)

	# Execute the trajectory on the robot.
	if traj != None:
	    robot.GetController().SetPath(traj)

    waitrobot(robot)

    raw_input("Press enter to exit...")

