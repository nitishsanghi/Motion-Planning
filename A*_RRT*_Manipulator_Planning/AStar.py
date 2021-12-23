#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import threading
from numpy import *
from Queue import PriorityQueue
from openravepy.misc import InitOpenRAVELogging
from func_timeout import func_timeout, FunctionTimedOut
InitOpenRAVELogging() 

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

def tuckleftarms(env,robot):
    with env:
        robot.SetActiveDOFs([15, 16, 17, 18, 19, 20, 21])
        robot.SetActiveDOFValues([-0.5, 1., 0.0,-1, 0.0, 0.0, 0.0]);         
        robot.GetController().SetDesired(robot.GetDOFValues())
        raw_input('Enter any key to quit. ')
    waitrobot(robot)
def tuckrightarms(env,robot):
    with env:
        robot.SetActiveDOFs([27, 28, 29, 30, 31, 32, 33])
        robot.SetActiveDOFValues([-0.5, 1.5, 0.0, -2, 0.0, 0.0, 0.0]);         
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)    

def ConvertPathToTrajectory(robot,path=[]):
    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')    
    traj.Init(robot.GetActiveConfigurationSpecification())
    print(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(7),maxaccelerations=5*ones(7))
    return traj

    # Nodes coordinates
def xyz_nodes_coordinates(x_start_node, x_end_node, y_start_node, y_end_node, z_start_node, z_end_node):
        x_nodes = [x_start_node]
        y_nodes = [y_start_node]
        z_nodes = [z_start_node]

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
    
        while(z_nodes[-1]<=z_end_node):
            z_temp = round(z_nodes[-1]+step_z,2)
            if z_temp <= z_end_node:
                z_nodes.append(z_temp)
            else:
                break
        return x_nodes, y_nodes, z_nodes
    
    # Grid 3D list created in loop
def creategrid3d(x_nodes, y_nodes, z_nodes):
        xyz_grid = []
        for z_node in z_nodes:
            xy_grid = []
            for x_node in x_nodes:
                temp_grid_column = []
                for y_node in y_nodes:
                    temp_grid_column.append((x_node, y_node, z_node))
                xy_grid.append(temp_grid_column)
            xyz_grid.append(xy_grid)
        return xyz_grid

    # Movement steps for 8c
def movements8c():
        movements_8c = []
        for x in [-1,0,1]:
            for y in [-1,0,1]:
                for theta in [-1,0,1]:
                    movements_8c.append([theta, x, y])
        movements_8c.remove([0, 0, 0])
        return movements_8c

# Node Class
class Node:
        def __init__(self,parent,location,value):
            self.parent = parent
            self.location = location
            self.value = value
            self.h = 0.0
            self.g = 0.0
            self.fcost = 0.0
            self.configurations = []
        
        def f(self):
            self.fcost = self.g + self.h
        
        def costOfStep(self,node):
            dx = (self.value[0] - node.value[0])
            dy = (self.value[1] - node.value[1])
            dz = (self.value[2] - node.value[2])
            return dx*dx + dy*dy + dz*dz

# Convert grid points to nodes
def convertGridtoNode(xyz_grid): # Convert grid to nodes
        for i in range(len(xyz_grid)):
            for j in range(len(xyz_grid[0])):
                for k in range(len(xyz_grid[0][0])):
                    xyz_grid[i][j][k] = Node(None,(i,j,k) ,xyz_grid[i][j][k]) 
    
# Find closest start node
def findcloseststartnode(xyz_grid, start_position):
        x = start_position[0]
        y = start_position[1]
        z = start_position[2]
        done = False
        min_distance = 99999
        coordinates = [0, 0, 0]
        for z_coord in range(len(xyz_grid)):
            for x_coord in range(len(xyz_grid[0])):
                for y_coord in range(len(xyz_grid[0][0])):
                    temp_coord = xyz_grid[z_coord][x_coord][y_coord].value
                    dx2 = (temp_coord[0] - x)*(temp_coord[0] - x)
                    dy2 = (temp_coord[1] - y)*(temp_coord[1] - y)
                    dz2 = (temp_coord[2] - z)*(temp_coord[2] - z)
                    temp_distance = dx2 + dy2 + dz2
                    if temp_distance < min_distance:
                        min_distance = temp_distance
                        coordinates = [z_coord, x_coord, y_coord]
        return coordinates
    
# Find closest end node    
def findclosestendnode(xyz_grid, end_position):
        x = end_position[0]
        y = end_position[1]
        z = end_position[2]
        done = False
        min_distance = 99999
        coordinates = [0, 0, 0]
        for z_coord in range(len(xyz_grid)):
            for x_coord in range(len(xyz_grid[0])):
                for y_coord in range(len(xyz_grid[0][0])):
                    temp_coord = xyz_grid[z_coord][x_coord][y_coord].value
                    dx2 = (temp_coord[0] - x)*(temp_coord[0] - x)
                    dy2 = (temp_coord[1] - y)*(temp_coord[1] - y)
                    dz2 = (temp_coord[2] - z)*(temp_coord[2] - z)
                    temp_distance = dx2 + dy2 + dz2
                    if temp_distance < min_distance:
                        min_distance = temp_distance
                        coordinates = [z_coord, x_coord, y_coord]
        return coordinates

# Node collision checker
def nodecollisioncheck(config): # Check if node leads to collision of robot
        with env:
            robot.SetActiveDOFs(manip.GetArmIndices())
            robot.SetActiveDOFValues(config)
            #print(env.CheckCollision(robot))
            return env.CheckCollision(robot)

# Node constraint and collision check
def nodeconstraintcollisioncheck(child_node, parent_node):
        with env: # lock environment
            #print("checking collision and constraint")
            #robot.SetDOFValues(parent_node.configurations,manip.GetArmIndices())
            endeffector_position = child_node.value
            #print(endeffector_position)
            ikparam = IkParameterization(endeffector_position, ikmodel.iktype) # build up the translation3d ik query
            #sol = manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            sol = manip.FindIKSolution(ikparam, IkFilterOptions.IgnoreEndEffectorEnvCollisions)
            if  sol is not None:
                #print(sol)
                if not nodecollisioncheck(sol):
                    #print("IK valid")
                    child_node.configurations = sol
                    return False
                else:
                    #print("Collision Check Failed")
                    return True
            else:
                #print("IK not valid")
                return True

# Children 8 nodes 
def findchildren(node, movements, xyz_grid): #Find potential children for 4c
        #print "finding children"
        z_lim = len(xyz_grid) - 1
        x_lim = len(xyz_grid[0]) - 1
        y_lim = len(xyz_grid[0][0]) - 1
        children = []
        #print node.value
        for movement in movements:
            temp_z = node.location[0] + movement[0]
            temp_x = node.location[1] + movement[1]
            temp_y = node.location[2] + movement[2]
            if temp_z < 0 or temp_x < 0 or temp_y < 0:
                #print "Out of bounds"
                continue
            if temp_z >  z_lim or temp_x > x_lim or temp_y > y_lim:
                #print "Out of bounds"
                continue
            """if nodecollisioncheck(xytheta_grid[temp_theta][temp_x][temp_y]):
                #print "Collision"
                continue"""
            children.append(xyz_grid[temp_z][temp_x][temp_y])
            #print "Valid child found"
        return children

# Euclidean distance
def heuristic(node, goalconfig):
        dx = (node.value[0] - goalconfig[0])
        dy = (node.value[1] - goalconfig[1])
        dz = (node.value[2] - goalconfig[2])
        node.h =  dx*dx + dy*dy + dz*dz

# Path found
def pathfound(xytheta_grid, current):
        path = []
        while current.parent:
            path.append(current)
            current = current.parent
        path.append(current)
        return path

def pathfinder(goalconfig, startconfig, xyz_grid, movements):
        start_time = time.clock()
        open_list = set()
        closed_list = set()
        start_coord = findcloseststartnode(xyz_grid, startconfig)
        end_coord = findclosestendnode(xyz_grid, goalconfig)
        start = xyz_grid[start_coord[0]][start_coord[1]][start_coord[2]]
        ikparam = IkParameterization(start.value, ikmodel.iktype) # build up the translation3d ik query
        sol = manip.FindIKSolution(ikparam, IkFilterOptions.IgnoreEndEffectorEnvCollisions)
        start.configurations = sol
        end = xyz_grid[end_coord[0]][end_coord[1]][end_coord[2]]
        end_goalconfig = end.value
        heuristic(start, end_goalconfig)
        current = start
        open_list.add(current)
        while open_list:
            #print(len(open_list))
            #current = min(open_list, key=lambda o:o.fcost)
            open_queue = PriorityQueue()
            for node in open_list:
                open_queue.put((node.fcost, node))
            current = open_queue.get_nowait()
            current = current[1]
            if current.value == (end_goalconfig[0], end_goalconfig[1], end_goalconfig[2]):
                print "Goal Found"
                current.f()
                #print current.g
                #print current.fcost
                #print time.clock() - start_time
                return pathfound(xyz_grid, current)

            open_list.remove(current)
            closed_list.add(current) 
            
            robot.SetDOFValues(current.configurations,manip.GetArmIndices())
            
            children = findchildren(current, movements, xyz_grid)
            for child in children:
                if child in closed_list:
                    continue
                if child in open_list and not nodecollisioncheck(child.configurations):
                    temp_g = current.g + child.costOfStep(current)
                    if child.g > temp_g:
                        child.g = temp_g
                        child.f()
                        child.parent = current
                else:
                    if nodeconstraintcollisioncheck(child, current):
                        child.g = 10000
                        child.f()
                        collision_config.add(child)
                    else:
                        collision_free_config.add(child)
                        child.g = current.g + child.costOfStep(current)
                        heuristic(child, goalconfig)
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
    env.Load('finalproject2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    
    manip = robot.SetActiveManipulator('leftarm') # set the manipulator to leftarm
    # tuck in the PR2's arms for driving
    tuckrightarms(env,robot)
    tuckleftarms(env,robot)
    #Inverse Kinematic Model
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    Transform_EndEffector = manip.GetEndEffectorTransform()
    startconfig = Transform_EndEffector[0:3,3]
    
    goalconfig = [-.2425, 0.632, 0.9123]
    #print(Transform_EndEffector)
    print(startconfig)
    #print(manip.GetArmIndices())
    
    #print(robot.GetActiveDOFValues())
    print(robot.GetDOFValues(manip.GetArmIndices()))
    start_configuration = robot.GetDOFValues(manip.GetArmIndices())
    raw_input("Press enter to exit...")
    start = time.clock()
    #### YOUR CODE HERE ####
    ### Grid Creation
    # Start Node
    x_start_node = -.8
    y_start_node = -.2
    z_start_node = 0.4

    # Start node coordinate
    theta_start_coord = 2

    # End Node
    x_end_node = .2
    y_end_node = .8
    z_end_node = 1.0

    # Step Size
    step_x = .02
    step_y = .02
    step_z = .02
    
    x_nodes, y_nodes, z_nodes = xyz_nodes_coordinates(x_start_node, x_end_node, y_start_node, y_end_node, z_start_node, z_end_node) #Finding node coordinates        
    xyz_grid = creategrid3d(x_nodes, y_nodes, z_nodes) # Creating grid
    movements = movements8c()
    convertGridtoNode(xyz_grid)
    coordinates_start = findcloseststartnode(xyz_grid, startconfig)
    coordinates_end = findclosestendnode(xyz_grid, goalconfig)

    x_nodes_num = len(x_nodes)
    y_nodes_num = len(y_nodes)
    z_nodes_num = len(z_nodes)
    
    print("Number of nodes")
    print(len(x_nodes)*len(y_nodes)*len(z_nodes))

    collision_free_config = set()
    collision_config = set()

    time_axis = []
    cost_axis = []
    start_time = 0
    #### Draw the X and Y components of the configurations explored by your algorithm    
    path_node = pathfinder(goalconfig, startconfig, xyz_grid,movements)
    end = time.clock()
    print "Solution Time: ", end - start
    spinner = None
    try:
        handles = []
        xyz_coord = []
        for x_node in x_nodes:
            for y_node in y_nodes:
                for y_node in y_nodes:
                    for z_node in z_nodes:
                        xyz_coord.append([x_node, y_node, z_node])
        for i in range(1,len(path_node)):
            point1 = (path_node[i-1].value[0], path_node[i-1].value[1], path_node[i-1].value[2])
            point2 = (path_node[i].value[0], path_node[i].value[1], path_node[i].value[2])
            handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((0,0,1)))))
        spinner = PlotSpinner(handles[-1])
        spinner.start()
        raw_input('Enter any key to quit. ')
        handles = None
    finally:
           if spinner is not None:
               spinner.ok = False

    path = [] #put your final path in this variable
    configs = []
    if path_node:
        for node in path_node:
            path.append(node.value)
            configs.append(node.configurations)
    else:
        print "No Solution Found"
        
    path.append(startconfig)
    configs.append(start_configuration)
    path.reverse()
    configs.reverse()
    ikparam = IkParameterization(goalconfig, ikmodel.iktype) # build up the translation3d ik query
    sol = manip.FindIKSolution(ikparam, IkFilterOptions.IgnoreEndEffectorEnvCollisions)
    #path.append(goalconfig)
    #configs.append(sol)
    print("Path found :")
    print(path)
    print(len(path))
    print("Configuraton founds")
    print(configs)
    print(len(configs))
    
    #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start
    raw_input("Press enter to exit...")
    # Now that you have computed a path, convert it to an openrave trajectory 

    traj = ConvertPathToTrajectory(robot, configs)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)

    waitrobot(robot)

    raw_input("Press enter to exit...")