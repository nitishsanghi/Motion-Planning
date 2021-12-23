# A* and Variants with OPENRAVE simulation environment

Path finding for mobile robots is a basic problem which can be solved using a informed search algorithm like A* or its variants. In this project A* and AN* are implemented in python and interfaced with OPENRAVE simulation environment. A* is guaranteed to find the shortest path between two points but can be time consuming. AN* is an iterative version of A* which quickly finds and returns a path and can iteratively improve upon the path to find the optimal path. The gif illustration below is for A* path found and simulated in OPENRAVE. 

![Alt Text](https://media.giphy.com/media/J2chJ2bex6Pca2SUEp/giphy.gif)


The gif below illustrates solution found by AN* variant. The variant finds a sub-optimal path faster than A* (the yellow path) and then iteratively works towards finding the optimal path. The advantage of AN* is that a feasible path can be found quickly and then iteratively with enough time a path which tends to be the optimal path. In the image on the right, in an obstacle free environment 3 paths are shown found by the variant algorithm. The yellow path is the sub-optimal path found quickly by the algorithm. The variant algorithm then iteratively refines the path to find paths which are closer to the optimal solution that A* would have found. The pink path found iteratively is better than the yellow path and finally the black path which is the closest to the optimal solution path.

![Alt Text](https://media.giphy.com/media/seyAkWbwzZu7DAf7jK/giphy.gif)<img src="https://github.com/nitishsanghi/Motion-Planning/blob/master/A*Variants/ANStarSample.jpg" width="500" height="250">
