# 7-DOF Manipulator A*-RRT* based path planner 

In this project, a motion planner is developed composed of A* and RRT* based planners for a 7-DOF manipulator. First, a path is found for the end effector in the task space using an A* in low dimensional space. The path found is then used to initialize an RRT* based planner to find manipulator path in high dimensional C-space. As a baseline BiRRT with path smoothing is used. 

The gif below illustrates the path found using BiRRT which has been smoothened. 
![Alt Text](https://media.giphy.com/media/qVzJpsxeNL6NegGH2y/giphy.gif)


The gifs below illustrate the paths found for the end effector using A* which is then used to initialize the path to be iteratively optimized by RRT* planner as illustrated in the next gif.

![Alt Text](https://media.giphy.com/media/16FVZg4TPePu27YHED/giphy.gif)

The blue path is the end effector path found in task space. Using inverse kinematics, the 7-DOF path for initializing the RRT* planner is found from the initial end effector path. RRT* find the path in red which is smoother than the A* path found initially.
![Alt Text](https://media.giphy.com/media/JthDcu2cWZPCSRthEh/giphy.gif)
