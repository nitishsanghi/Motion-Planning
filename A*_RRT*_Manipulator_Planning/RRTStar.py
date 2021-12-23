#!/usr/bin/env python
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
        print(path[i])
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(7),maxaccelerations=5*ones(7))
    return traj

class RRTStar:
        
        class Node:
            def __init__(self, parent, configuration, position):
                self.parent = parent
                self.configuration = configuration
                self.position = position
                self.cost = 0.0 

        def __init__(self, configurations, positions, lower, upper):
            self.configurations = configurations
            self.positions = positions
            self.tree = []
            self.start_node = None
            self.goal_node = None
            self.upper = upper
            self.lower = lower

        def roundvariable(self, variable):
            return [round(v,3) for v in variable]
        
        def initialize_tree(self):
            print("Initialize tree")
            configuration = self.roundvariable(self.configurations.pop(0))
            position = self.roundvariable(self.positions.pop(0))
            node = self.Node(None, configuration, position)
            self.tree.append(node)
            while(self.configurations):
                configuration = self.roundvariable(self.configurations.pop(0))
                position = self.roundvariable(self.positions.pop(0))
                node = self.Node(node, configuration, position)
                node.cost = self.node_dist(node, node.parent) + node.parent.cost
                self.tree.append(node)
    
        def node_dist(self, node_1, node_2):
            weights = [1, 1, 1, 1, 1, 1, 1]
            distance = 0
            pos_dist = 0
            config_1 = node_1.configuration
            config_2 = node_2.configuration
            position_1 = node_1.position
            position_2 = node_2.position
            for a,b,w in zip(config_1, config_2, weights):
                distance = distance + w*(a - b)*(a - b)
            distance = round(sqrt(round(distance,2)),2)
            cost = distance + pos_dist
            return cost
        
        def start_goal_nodes(self):
            print("Start goal nodes")
            self.start_node = self.tree[0]
            self.goal_node = self.tree[-1]
            print("Goal Initial Cost")
            print(self.goal_node.cost)
    
        def generate_path(self, goal_node):
            #print("Generate path")
            path = []
            node = self.goal_node
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
    
        def select_path_node(self):
            #print("Select path nodes")
            path = self.generate_path(self.goal_node)
            path.remove(self.start_node)
            path.remove(self.goal_node)
            node_id = random.randint(0,len(path))
            return path[node_id]
    
        def doflimiter(self, config):
            for i in range(len(config)):
                if config[i] < self.lower[i]:
                    config[i] = self.lower[i]
                if config[i] > self.upper[i]:
                    config[i] = self.upper[i] 
            return config
        
        def node_position(self, config):
            with env: # lock environment
                with robot: # save robot state
                    robot.SetDOFValues(config,manip.GetArmIndices()) # set the current solution
                    Transform_EndEffector = manip.GetEndEffectorTransform()
                    position = Transform_EndEffector[0:3,3]
            return position
            
        def generate_random_node(self):
            #print("Generate random node")
            path_node = self.select_path_node()
            mean_config = array(path_node.configuration)
            sigma = 0.1
            random_config = random.multivariate_normal(mean_config, sigma*sigma*identity(7))
            random_config = self.roundvariable(random_config)
            random_config = self.doflimiter(random_config)
            if not self.nodecollisioncheck(random_config):
                position = self.node_position(random_config)
                return self.Node(None, random_config, position)
            else:
                return self.generate_random_node()
         
        def nearest_node(self,random_node):
            distance_list = [self.node_dist(node, random_node) for node in self.tree]
            id = distance_list.index(min(distance_list))
            return self.tree[id]

        def straightlinecollision(self, node1, node2):
            delta_config = array(node2.configuration) - array(node1.configuration)
            distance = 0
            for a,b in zip(node1.configuration, node2.configuration):
                distance = distance + (a - b)*(a - b)
            distance = sqrt(distance)
            vector = delta_config
            for k in range(1,10):
                temp_config = []
                for j in range(7):
                    step = 0.1*k*(node2.configuration[j] - node1.configuration[j])/distance
                    temp_config.append(node1.configuration[j] + step)
                temp_config = self.doflimiter(temp_config)
                if self.nodecollisioncheck(temp_config):# or self.nodeselfcollisioncheck(temp_config):
                    return True
            return False
        
        def select_parent(self, new_node, near_nodes, nearest_node):
            #print("Select parents")
            if not near_nodes:
                return None
            min_cost = nearest_node.cost + self.node_dist(new_node, nearest_node)
            min_node = nearest_node
            for node in near_nodes:
                if not self.straightlinecollision(node, new_node) and node.cost + self.node_dist(node, new_node) < min_cost:
                    min_cost = node.cost + self.node_dist(node, new_node)
                    min_node = node
            new_node.parent = min_node
            new_node.cost = min_cost
            return new_node
    
        def find_near_nodes(self, new_node):
            #print("Finding near nodes")
            radius = 1.0
            distance_list = [self.node_dist(node, new_node) for node in self.tree]
            near_ids = [distance_list.index(diff) for diff in distance_list if diff <= radius]
            nodes = [self.tree[i] for i in near_ids]
            return nodes

        def rewire(self, new_node, near_nodes):
            #print("Rewire")
            if not near_nodes:
                return None
            for node in near_nodes:
                if not self.straightlinecollision(new_node, node) and new_node.cost + self.node_dist(node, new_node) < node.cost:
                    node.parent = new_node
                    self.propagate_cost_to_leaves(new_node)
            
        def propagate_cost_to_leaves(self, parent_node):
            for node in self.tree:
                if node.parent == parent_node:
                    node.cost = parent_node.cost + self.node_dist(parent_node, node)
                    self.propagate_cost_to_leaves(node)
    
        def RRTStarPlanning(self):
            print("RRT Star planning")
            #print(len(configurations))
            self.initialize_tree()
            #print(len(self.tree))
            self.start_goal_nodes()
            print("Straight line cost")
            print(self.node_dist(self.start_node, self.goal_node))
            for k in range(1000):
                #print(self.goal_node.cost)
                #print(k)
                new_node = self.generate_random_node()
                near_node = self.nearest_node(new_node)
                near_nodes = self.find_near_nodes(new_node)
                temp_node = self.select_parent(new_node, near_nodes, near_node)
                if temp_node is not None:
                    new_node = temp_node
                else:
                    new_node.parent = near_node
                self.tree.append(new_node)
                near_nodes.remove(near_node)
                self.rewire(new_node, near_nodes)
                if len(self.generate_path(self.goal_node)) < 10:
                    break
            print("Goal cost")
            print(self.goal_node.cost)
            return self.generate_path(self.goal_node)

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
    doflowerlimits, dofupperlimits = robot.GetActiveDOFLimits()
    doflowerlimits[4] = - math.pi
    doflowerlimits[6] = - math.pi
    dofupperlimits[4] = math.pi
    dofupperlimits[6] = math.pi
    raw_input("Press enter to exit...")
    start_configuration = robot.GetDOFValues(manip.GetArmIndices())
    start = time.clock()
    #### YOUR CODE HERE ####
    ### Grid Creation
    # Start Node      
    positions = [array([-0.13369809, -0.06331751,  0.49355663]), (-0.14, -0.06, 0.5), (-0.16, -0.04, 0.48), (-0.18, -0.02, 0.46), (-0.2, -0.02, 0.46), (-0.22, 0.0, 0.44), (-0.22, 0.02, 0.42), (-0.22, 0.02, 0.4), (-0.24, 0.04, 0.42), (-0.24, 0.06, 0.4), (-0.24, 0.08, 0.4), (-0.26, 0.08, 0.42), (-0.26, 0.08, 0.44), (-0.26, 0.1, 0.46), (-0.28, 0.12, 0.48), (-0.28, 0.14, 0.5), (-0.28, 0.16, 0.52), (-0.28, 0.18, 0.54), (-0.3, 0.2, 0.56), (-0.32, 0.2, 0.56), (-0.34, 0.2, 0.58), (-0.36, 0.2, 0.6), (-0.36, 0.22, 0.62), (-0.38, 0.22, 0.64), (-0.4, 0.22, 0.66), (-0.42, 0.24, 0.68), (-0.44, 0.26, 0.7), (-0.44, 0.24, 0.72), (-0.46, 0.26, 0.74), (-0.48, 0.28, 0.76), (-0.5, 0.28, 0.74), (-0.52, 0.28, 0.76), (-0.52, 0.3, 0.76), (-0.5, 0.32, 0.74), (-0.48, 0.34, 0.76), (-0.46, 0.36, 0.78), (-0.44, 0.38, 0.8), (-0.44, 0.4, 0.8), (-0.44, 0.42, 0.82), (-0.44, 0.44, 0.84), (-0.42, 0.46, 0.86), (-0.4, 0.48, 0.84), (-0.4, 0.5, 0.82), (-0.38, 0.52, 0.84), (-0.36, 0.54, 0.86), (-0.34, 0.56, 0.88), (-0.32, 0.58, 0.9), (-0.3, 0.6, 0.9), (-0.28, 0.62, 0.9), (-0.26, 0.62, 0.9), (-0.24, 0.62, 0.9), (-0.24, 0.62, 0.92), (-0.24, 0.64, 0.92)]

    configurations = [array([-5.00000000e-01,  1.00000000e+00,  6.66133815e-16, -1.00000000e+00,
        0.00000000e+00, -1.00000036e-01,  0.00000000e+00]), array([-0.07786671,  0.85195633,  0.87915826, -1.        ,  0.        ,
       -0.10000004,  0.        ]), array([-1.76001921e-01,  1.01569423e+00,  5.75753221e-01, -1.00000000e+00,
        1.83995210e-16, -3.00000036e-01, -1.47196168e-16]), array([-7.65038788e-02,  1.04209953e+00,  7.22095608e-01, -1.00000000e+00,
       -1.53636000e-16, -4.00000036e-01, -1.47196168e-16]), array([-2.71425788e-02,  1.04206311e+00,  8.24782438e-01, -1.00000000e+00,
       -1.25116743e-16, -5.00000036e-01, -2.19567617e-16]), array([-2.55785479e-02,  1.12399051e+00,  7.65350561e-01, -1.00000000e+00,
        0.00000000e+00, -6.00000036e-01, -1.57009246e-16]), array([-7.38876369e-02,  1.19189221e+00,  6.17725841e-01, -1.00000000e+00,
        0.00000000e+00, -6.00000036e-01, -1.47196168e-16]), array([ 2.23502768e-03,  1.16954614e+00,  7.58853127e-01, -1.00000000e+00,
        0.00000000e+00, -5.00000036e-01, -1.96261557e-16]), array([ 1.11644963e-01,  1.15506765e+00,  9.09957371e-01, -1.00000000e+00,
        1.42289629e-16, -7.00000036e-01,  1.47196168e-16]), array([ 0.03946809,  1.24483465,  0.70835577, -1.        ,  0.        ,
       -0.70000004,  0.        ]), array([ 2.03341999e-01,  1.18488937e+00,  9.54491041e-01, -1.00000000e+00,
        1.28796647e-16, -7.00000036e-01,  1.66822324e-16]), array([ 7.52865428e-02,  1.26540345e+00,  7.23784654e-01, -1.00000000e+00,
        0.00000000e+00, -9.00000036e-01,  2.50233486e-16]), array([ 2.92966067e-01,  1.07018864e+00,  1.18348282e+00, -1.00000000e+00,
        0.00000000e+00, -9.00000036e-01,  2.50233486e-16]), array([ 3.23191763e-01,  1.04709027e+00,  1.19539123e+00, -1.00000000e+00,
        0.00000000e+00, -1.00000004e+00,  2.30722327e-16]), array([ 2.92211465e-01,  1.11813041e+00,  1.06393023e+00, -1.00000000e+00,
        0.00000000e+00, -1.20000004e+00,  3.48364264e-16]), array([ 2.89268279e-01,  1.11337847e+00,  1.01174597e+00, -1.00000000e+00,
        0.00000000e+00, -1.30000004e+00,  2.45326947e-16]), array([ 2.71031281e-01,  1.11314918e+00,  9.32050988e-01, -1.00000000e+00,
        0.00000000e+00, -1.40000004e+00,  5.78971594e-16]), array([ 2.36652710e-01,  1.11412118e+00,  8.24890761e-01, -1.00000000e+00,
        0.00000000e+00, -1.50000004e+00,  3.53884121e-16]), array([ 1.86159185e-01,  1.11193615e+00,  7.15228324e-01, -1.00000000e+00,
        0.00000000e+00, -1.70000004e+00,  9.09856313e-16]), array([ 2.03884879e-01,  1.09666065e+00,  7.81235731e-01, -1.00000000e+00,
        0.00000000e+00, -1.80000004e+00,  1.00093394e-15]), array([ 1.54453708e-01,  1.03765196e+00,  7.66317078e-01, -1.00000000e+00,
        0.00000000e+00, -2.00000004e+00,  8.36564888e-16]), array([ 1.86746724e-01,  1.03149799e+00,  8.30609992e-01, -1.10000000e+00,
        0.00000000e+00, -2.00000004e+00,  6.32943522e-16]), array([ 2.07798241e-01,  1.08605772e+00,  7.39926512e-01, -1.20000000e+00,
       -1.21590168e-16, -1.90000004e+00,  5.64251977e-16]), array([ 1.64677391e-01,  1.11796335e+00,  6.61660843e-01, -1.30000000e+00,
        1.76635402e-16, -1.90000004e+00,  7.21261223e-16]), array([ 8.67844744e-02,  1.15218867e+00,  5.34178304e-01, -1.40000000e+00,
        0.00000000e+00, -1.90000004e+00,  8.63550852e-16]), array([ 1.26717230e-01,  1.15346148e+00,  5.42816445e-01, -1.50000000e+00,
        0.00000000e+00, -1.90000004e+00,  8.04672385e-16]), array([ 1.28806519e-01,  1.15880984e+00,  4.84777329e-01, -1.60000000e+00,
        0.00000000e+00, -1.90000004e+00,  7.85046229e-16]), array([ 3.30005331e-01,  1.00645807e+00,  9.04705355e-01, -1.60000000e+00,
        0.00000000e+00, -1.80000004e+00,  4.80840815e-16]), array([ 1.08343959e-01,  1.07328584e+00,  4.77921022e-01, -1.70000000e+00,
        0.00000000e+00, -1.90000004e+00,  6.67289295e-16]), array([-3.87679304e-02,  1.07818649e+00,  1.55403947e-01, -1.80000000e+00,
        0.00000000e+00, -1.90000004e+00,  7.45793918e-16]), array([ 3.51837896e-01,  1.00626276e+00,  8.67428102e-01, -1.80000000e+00,
        0.00000000e+00, -1.90000004e+00,  3.53270803e-16]), array([ 2.00553884e-01,  1.03611792e+00,  6.21735292e-01, -1.90000000e+00,
        0.00000000e+00, -1.90000004e+00,  3.33644647e-16]), array([ 3.06094372e-01,  1.01010706e+00,  7.09499253e-01, -1.90000000e+00,
        0.00000000e+00, -1.90000004e+00,  3.43457725e-16]), array([ 6.11477074e-01,  1.27427982e+00,  9.35523611e-01, -1.90000000e+00,
        0.00000000e+00, -1.50000004e+00,  1.42289629e-16]), array([ 0.85836465,  1.08576944,  1.18920866, -1.9       ,  0.        ,
       -1.30000004,  0.        ]), array([ 0.89576329,  1.01470581,  1.15296107, -1.9       ,  0.        ,
       -1.20000004,  0.        ]), array([ 0.9076778 ,  0.97867001,  1.08513342, -1.9       ,  0.        ,
       -1.10000004,  0.        ]), array([ 0.91965611,  1.02023862,  1.03165803, -1.9       ,  0.        ,
       -1.10000004,  0.        ]), array([ 0.91944475,  0.97460937,  0.96336806, -1.9       ,  0.        ,
       -1.10000004,  0.        ]), array([ 0.88755425,  0.96042616,  0.84984404, -1.9       ,  0.        ,
       -1.10000004,  0.        ]), array([ 0.78319073,  0.9972442 ,  0.64980075, -1.9       ,  0.        ,
       -1.00000004,  0.        ]), array([ 0.71733439,  1.10748006,  0.52121817, -1.9       ,  0.        ,
       -0.90000004,  0.        ]), array([ 0.60582147,  1.1982038 ,  0.32097181, -1.9       ,  0.        ,
       -0.90000004,  0.        ]), array([ 0.84473259,  1.08333691,  0.60023761, -1.9       ,  0.        ,
       -0.70000004,  0.        ]), array([ 0.59621881,  1.09477281,  0.24478031, -1.9       ,  0.        ,
       -0.60000004,  0.        ]), array([ 0.76780803,  0.99472495,  0.44191816, -1.9       ,  0.        ,
       -0.40000004,  0.        ]), array([ 0.82062236,  0.90716756,  0.48961504, -1.9       ,  0.        ,
       -0.20000004,  0.        ]), array([ 6.06557743e-01,  9.35492619e-01,  1.19687446e-01, -1.90000000e+00,
        1.30000000e+00, -2.00000036e-01,  1.17756934e-16]), array([ 6.31833485e-01,  8.43848087e-01,  1.88377828e-02, -1.90000000e+00,
        2.00000000e+00, -6.00000036e-01, -5.29906205e-16]), array([ 7.15016221e-01,  7.68879103e-01,  1.77903185e-01, -1.90000000e+00,
        2.30000000e+00, -7.00000036e-01, -4.90653893e-16]), array([ 6.07384078e-01,  6.95125828e-01,  4.09159900e-02, -1.90000000e+00,
        2.50000000e+00, -1.00000004e+00, -4.84520720e-16]), array([ 6.97107175e-01,  6.23833067e-01,  2.10304353e-01, -1.90000000e+00,
        2.60000000e+00, -1.00000004e+00, -4.79365021e-16]), array([ 7.05887153e-01,  6.16841370e-01,  2.49451554e-01, -1.90000000e+00,
        2.80000000e+00, -1.00000004e+00, -4.71027738e-16])]
    

    print("Number of initial configuration nodes")
    print(len(configurations))
    rrtstar = RRTStar(configurations, positions, doflowerlimits, dofupperlimits)
    configuration_nodes = rrtstar.RRTStarPlanning()
    end = time.clock()
    print "Time: ", end - start
    print("Number of final configuration nodes")
    print(len(configuration_nodes))
 
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
            handles.append(env.plot3(points=array((node[0],node[1],node[2])), pointsize=10.0, colors=array(((0,0,0)))))
        
        for i in range(1,len(positions_rrt)):
            point1 = (positions_rrt[i-1][0], positions_rrt[i-1][1], positions_rrt[i-1][2])
            point2 = (positions_rrt[i][0], positions_rrt[i][1], positions_rrt[i][2])
            handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((1,0,0)))))
        positions = [array([-0.13369809, -0.06331751,  0.49355663]), (-0.14, -0.06, 0.5), (-0.16, -0.04, 0.48), (-0.18, -0.02, 0.46), (-0.2, -0.02, 0.46), (-0.22, 0.0, 0.44), (-0.22, 0.02, 0.42), (-0.22, 0.02, 0.4), (-0.24, 0.04, 0.42), (-0.24, 0.06, 0.4), (-0.24, 0.08, 0.4), (-0.26, 0.08, 0.42), (-0.26, 0.08, 0.44), (-0.26, 0.1, 0.46), (-0.28, 0.12, 0.48), (-0.28, 0.14, 0.5), (-0.28, 0.16, 0.52), (-0.28, 0.18, 0.54), (-0.3, 0.2, 0.56), (-0.32, 0.2, 0.56), (-0.34, 0.2, 0.58), (-0.36, 0.2, 0.6), (-0.36, 0.22, 0.62), (-0.38, 0.22, 0.64), (-0.4, 0.22, 0.66), (-0.42, 0.24, 0.68), (-0.44, 0.26, 0.7), (-0.44, 0.24, 0.72), (-0.46, 0.26, 0.74), (-0.48, 0.28, 0.76), (-0.5, 0.28, 0.74), (-0.52, 0.28, 0.76), (-0.52, 0.3, 0.76), (-0.5, 0.32, 0.74), (-0.48, 0.34, 0.76), (-0.46, 0.36, 0.78), (-0.44, 0.38, 0.8), (-0.44, 0.4, 0.8), (-0.44, 0.42, 0.82), (-0.44, 0.44, 0.84), (-0.42, 0.46, 0.86), (-0.4, 0.48, 0.84), (-0.4, 0.5, 0.82), (-0.38, 0.52, 0.84), (-0.36, 0.54, 0.86), (-0.34, 0.56, 0.88), (-0.32, 0.58, 0.9), (-0.3, 0.6, 0.9), (-0.28, 0.62, 0.9), (-0.26, 0.62, 0.9), (-0.24, 0.62, 0.9), (-0.24, 0.62, 0.92), (-0.24, 0.64, 0.92)]
        for node in positions:
            handles.append(env.plot3(points=array((node[0],node[1],node[2])), pointsize=10.0, colors=array(((0,0,0)))))
        
        for i in range(1,len(positions)):
            point1 = (positions[i-1][0], positions[i-1][1], positions[i-1][2])
            point2 = (positions[i][0], positions[i][1], positions[i][2])
            handles.append(env.drawlinestrip(points=array((point1,point2)), linewidth=7.0,colors = array(((0,0,1)))))

        spinner = PlotSpinner(handles[-1])
        spinner.start()
        raw_input('Enter any key to quit. ')
        handles = None
    finally:
           if spinner is not None:
               spinner.ok = False
    
        #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 

    traj = ConvertPathToTrajectory(robot, configs)
    print(traj.GetDuration())
    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")
