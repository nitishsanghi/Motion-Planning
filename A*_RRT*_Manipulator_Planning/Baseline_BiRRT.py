from openravepy import *
# -*- coding: utf-8 -*-
import time
import openravepy
import threading
import random
import math
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
        robot.SetActiveDOFValues([-0.5, 1.0, 0.0, -1, 0.0, 0.0, 0.0]);         
        robot.GetController().SetDesired(robot.GetDOFValues())
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
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(7),maxaccelerations=5*ones(7))
    return traj

class BiRRT:    
    class Node:
        def __init__(self, parent, configuration):
                self.parent = parent
                self.configuration = configuration
                self.cost = 0.0 

    def __init__(self, configurations, lower, upper):
            self.configurations = configurations
            self.start_node = None
            self.goal_node = None
            self.upper = upper
            self.lower = lower
            self.current_node = None
            self.current_tree = None
            self.current_config = None
            self.goal_node_temp = None

    def roundvariable(self, variable):
            return [round(v,3) for v in variable]
        
    def initialize_tree(self, tree, node):
            print("Initialize tree")
            tree.append(node)
            return tree
    
    def node_dist(self, node_1, node_2):
            weights = [1, 1, 1, 1, 1, 1, 1]
            distance = 0
            config_1 = node_1.configuration
            config_2 = node_2.configuration
            for a,b,w in zip(config_1, config_2, weights):
                distance = distance + w*(a - b)*(a - b)
            distance = round(sqrt(round(distance,2)),2)
            return distance
    
    def start_goal_nodes(self):
            print("Start goal nodes")
            self.start_node = self.Node(None, self.configurations[0])
            self.goal_node = self.Node(None, self.configurations[-1])
            
    def generate_path(self, node):
            #print("Generate path")
            path = []
            while node.parent is not None:
                path.append(node)
                node = node.parent
            if node.parent is None:
                path.append(node)
            path.reverse()
            return path
            
    def nodecollisioncheck(self, config):
            #print("Checking collision")
            with env:
                robot.SetActiveDOFs(manip.GetArmIndices())
                robot.SetActiveDOFValues(config)
                return env.CheckCollision(robot)
            
    def nodeselfcollisioncheck(self, config):
            #print("Checking collision")
            with robot:
                robot.SetActiveDOFs(manip.GetArmIndices())
                robot.SetActiveDOFValues(config)
                return robot.CheckSelfCollision()
    
    def doflimiter(self, config):
            for i in range(len(config)):
                if config[i] < self.lower[i]:
                    config[i] = self.lower[i]
                if config[i] > self.upper[i]:
                    config[i] = self.upper[i] 
            return config
            
    def generate_random_node(self):
            #print("Generate random node")
            random_config = []
            for i in range(7):
                random_config.append(random.uniform(self.lower[i], self.upper[i]))
            
            random_config = self.roundvariable(random_config)
            random_config = self.doflimiter(random_config)
            if not self.nodecollisioncheck(random_config) and not self.nodeselfcollisioncheck(random_config):
                return self.Node(None, random_config)
            else:
                #print("Finding Valid Node")
                return self.generate_random_node()
        
        #print(generate_random_node()) 
    def nearest_node(self, tree, random_node):
            distance_list = [self.node_dist(node, random_node) for node in tree]
            id = distance_list.index(min(distance_list))
            return tree[id]

    def new_config(self, near_node, random_node):
            distance = self.node_dist(near_node, random_node)
            temp_config = []
            for j in range(7):
                step = 0.1*(random_node.configuration[j] - near_node.configuration[j])/distance
                temp_config.append(near_node.configuration[j] + step)
            temp_config = self.doflimiter(temp_config)
            #print(temp_config)
            #print(near_node.configuration)
            #print(random_node.configuration)
            if self.nodecollisioncheck(temp_config) or self.nodeselfcollisioncheck(temp_config):
                return False
            else:
                self.current_config = temp_config
                return True

    def extend(self, tree, random_node):
            #print("Extending")
            near_node = self.nearest_node(tree, random_node)
            if self.new_config(near_node, random_node):
                parent_node = near_node
                self.current_node = self.Node(parent_node, self.current_config)
                tree.append(self.current_node)
                if self.node_dist(self.current_node, random_node) < 0.1:
                    self.goal_node_temp = random_node
                    return "reached"
                else:
                    return "advanced"
            return "trapped"
    
    def connect(self, tree, new_node):
            #print("Connecting")
            flag = "advanced"
            while flag == "advanced":
                flag = self.extend(tree, new_node)
            return flag
    
    def swap(self, tree1, tree2):
            temp_tree = tree1
            tree1 = tree2
            tree2 = temp_tree
            return tree1, tree2

    def smoothen(self, path):
            configs = []
            if path:
                for node in path:
                    configs.append(node.configuration)
            else:
                print("No Solution found")
                return None
            
            while len(configs)>20:
                id1 = random.randint(1,len(configs)-2)
                id2 = id1
                while id2 <= id1:
                    id2 = random.randint(id1+1,len(configs)-1)
                flag = True
                distance = self.node_dist(path[id1], path[id2])
                for k in range(1,1000):
                    temp_config = []
                    for j in range(7):
                        step = 0.001*k*(path[id2].configuration[j] - path[id1].configuration[j])/distance
                        temp_config.append(path[id1].configuration[j] + step)
                    temp_config = self.doflimiter(temp_config)
                    if self.nodecollisioncheck(temp_config) or self.nodeselfcollisioncheck(temp_config):
                        flag = False
                        break
                if flag:
                    configs = configs[0:id1+1] + configs[id2:len(configs)]
                    path = path[0:id1+1] + path[id2:len(path)]
            return path
            
    def RRTBiPlanning(self):
            print("BiRRT planning")
            self.start_goal_nodes()
            tree_a = []
            tree_b = []
            tree_a = self.initialize_tree(tree_a, self.start_node)
            tree_b = self.initialize_tree(tree_b, self.goal_node)
            #print("Tree")
            #print(tree_a)
            #print(tree_b)
            for i in range(10000):
                random_node = self.generate_random_node()
                #print(tree_a)
                #print(tree_b)
                if not (self.extend(tree_a, random_node) == "trapped"):
                    if self.connect(tree_b, self.current_node) == "reached":
                        print("Path found")
                        a = self.generate_path(self.current_node)
                        b = self.generate_path(self.goal_node_temp)
                        #print(len(a))
                        #print(len(b))
                        if a[0] == self.start_node:
                            b.reverse()
                            return a + b
                        else:
                            a.reverse()
                            return b + a
                tree_a, tree_b = self.swap(tree_a, tree_b)


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
    #print(startconfig)
    #print(manip.GetArmIndices())
    doflowerlimits, dofupperlimits = robot.GetActiveDOFLimits()
    doflowerlimits[4] = - math.pi
    doflowerlimits[6] = - math.pi
    dofupperlimits[4] = math.pi
    dofupperlimits[6] = math.pi
    #print(robot.GetActiveDOFValues())
    #print(robot.GetDOFValues(manip.GetArmIndices()))
    start_configuration = robot.GetDOFValues(manip.GetArmIndices())
    start = time.clock()
    #### YOUR CODE HERE ####
    ### Grid Creation
    # Start Node
    positions = astar_configurations = [[-0.13369809, -0.06331751,  0.49355663], [-0.2425, 0.632, 0.9123]]
    
    configurations = [array([-5.00000000e-01,  1.00000000e+00,  6.66133815e-16, -1.00000000e+00,
        0.00000000e+00, -1.00000036e-01,  0.00000000e+00]), array([ 8.89404688e-01,  5.21771725e-01,  5.67576198e-01, -1.90000000e+00,
        2.80000000e+00, -1.00000004e+00, -1.57009246e-15])]



    rrt = BiRRT(configurations, doflowerlimits, dofupperlimits)
    configuration_nodes = rrt.RRTBiPlanning()
    print("Number of initial configuration nodes")
    print(len(configuration_nodes))
    """total_cost = 0
    for i in range(len(configuration_nodes)-2):
        weights = [1, 1, 1, 1, 1, 1, 1]
        distance = 0
        config_1 = configuration_nodes[i].configuration
        config_2 = configuration_nodes[i+1].configuration
        for a,b,w in zip(config_1, config_2, weights):
            distance = distance + w*(a - b)*(a - b)
        distance = round(sqrt(round(distance,2)),2)
        total_cost = total_cost + distance
    print("Initial Cost")
    print(total_cost)"""
    configuration_nodes = rrt.smoothen(configuration_nodes)
    end = time.clock()
    print "Algorithm Time: ", end - start
    print("Number of final configuration nodes")
    print(len(configuration_nodes))
    total_cost = 0
    for i in range(len(configuration_nodes)-2):
        weights = [1, 1, 1, 1, 1, 1, 1]
        distance = 0
        config_1 = configuration_nodes[i].configuration
        config_2 = configuration_nodes[i+1].configuration
        for a,b,w in zip(config_1, config_2, weights):
            distance = distance + w*(a - b)*(a - b)
        distance = round(sqrt(round(distance,2)),2)
        total_cost = total_cost + distance
    print("Final Cost")
    print(total_cost)
    #print(len(configuration_nodes))
    #for node in configuration_nodes:
    #    print(node.cost)
    """spinner = None
    try:
        handles = []
        xyz_coord = []
        for x_node in x_nodes:
            for y_node in y_nodes:
                for y_node in y_nodes:
                    for z_node in z_nodes:
                        xyz_coord.append([x_node, y_node, z_node])
        for node in xyz_coord:
            handles.append(env.plot3(points=array((node[0],node[1],node[2])), pointsize=5.0, colors=array(((0,0,0)))))
        for node in collision_config:
            handles.append(env.plot3(points=array((node.value[0],node.value[1],.05)), pointsize=3.0, colors=array(((1,0,0)))))
        for node in collision_free_config:
            handles.append(env.plot3(points=array((node.value[0],node.value[1],.05)), pointsize=3.0, colors=array(((0,0,1)))))
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
               spinner.ok = False"""
    
    #put your final path in this variable
    configs = []
    if configuration_nodes:
        for node in configuration_nodes:
            configs.append(node.configuration)
    else:
        print "No Solution Found"
        
    #print("Configuraton founds")
    #print(configs)
    positions_rrt = []
    with env: # lock environment
        with robot: # save robot state
            for config in configs: # go through every 10th solution
                robot.SetDOFValues(config,manip.GetArmIndices()) # set the current solution
                Transform_EndEffector = manip.GetEndEffectorTransform()
                positions_rrt.append(Transform_EndEffector[0:3,3])
    
    spinner = None
    try:
        handles = []
        for node in positions_rrt:
            handles.append(env.plot3(points=array((node[0],node[1],node[2])), pointsize=5.0, colors=array(((0,0,0)))))
        for i in range(1,len(positions_rrt)):
            point1 = (positions_rrt[i-1][0], positions_rrt[i-1][1], positions_rrt[i-1][2])
            point2 = (positions_rrt[i][0], positions_rrt[i][1], positions_rrt[i][2])
            handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((1,0,0)))))

        """for i in range(1,len(positions)):
            point1 = (positions[i-1][0], positions[i-1][1], positions[i-1][2])
            point2 = (positions[i][0], positions[i][1], positions[i][2])
            handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((0,0,1)))))
        """
        spinner = PlotSpinner(handles[-1])
        spinner.start()
        raw_input('Enter any key to quit. ')
        handles = None
    finally:
           if spinner is not None:
               spinner.ok = False
                
    with env: # lock environment
        with robot: # save robot state
            for config in configs: # go through every 10th solution
                robot.SetDOFValues(config,manip.GetArmIndices()) # set the current solution
                env.UpdatePublishedBodies() # allow viewer to update new robot
                time.sleep(.1)
                #raw_input('press any key')"""
    
        #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, configs)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")


manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
manipprob.MoveManipulator(goal=[ 8.89404688e-01,  5.21771725e-01,  5.67576198e-01, -1.90000000e+00,
        2.80000000e+00, -1.00000004e+00, -1.57009246e-15]) # call motion planner with goal joint angles
robot.WaitForController(0) # wait