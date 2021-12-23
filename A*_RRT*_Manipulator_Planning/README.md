# 7-DOF Manipulator A*-RRT* based path planner 

In this project, a motion planner is developed composed of A* and RRT* based planners for a 7-DOF manipulator. First, a path is found for the end effector in the task space using an A* in low dimensional space. The path found is then used to initialize an RRT* based planner to find manipulator path in high dimensional C-space. As a baseline BiRRT with path smoothing is used. 

In the 
![Alt Text](https://media.giphy.com/media/16FVZg4TPePu27YHED/giphy.gif)
![Alt Text](https://media.giphy.com/media/qVzJpsxeNL6NegGH2y/giphy.gif)
![Alt Text](https://media.giphy.com/media/JthDcu2cWZPCSRthEh/giphy.gif)
