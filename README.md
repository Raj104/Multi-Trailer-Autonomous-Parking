# Multi-Trailer-Autonomous-Parking

Parking Problem Description:
We want multi-trailer trucks to autonomously park in given spot without any collisions.
The multi-trailer problem comes under the class of path planning with non-holonomic constraints.

Background :
A non-holonomic constraint is a non-integrable constraint. In this problem they are mostly velocity constraints.
Multiple path-planning algorithms such as hybrid A*, RRT exist  to solve this problem
Fundamentally these solutions tend to fail as the number of trailers increase. 
A-star starts to fail because of higher dimensionality
While solutions like RRT have been proposed to handle higher dimensions, due to the kinematics of the trailer-truck, simply applying RRT would suggest trajectories which would be very difficult for the trailer – truck system to execute

Methodology:
Implement hybrid RRT- local A-star using Reeds Shepp distance metric to generate the tree to the goal.
To traverse the nodes of a tree local A-star will be used
While sampling for RRT, with a probability of 0.01 the end goal will be sample.
After each node is sampled, nearest node in the tree will also be selected based on Reeds Shepp distance metric.
After a node is sampled, steer methodology will be used to generate multiple nodes from the nearest node to the sampled based on the assumed obstacle-less path from node to the nearest node. All the nodes which are not in collision will be added to the tree

Project Update:
We selected Pygame to create the simulation environment for the problem statement
Post having a sanity check by simulating a vehicle using the Ackerman steering model, we extended it to a single trailer model
The environment is a 250x250 grid with obstacles simulating a parking spot
First, we generated a path using just hybrid A –star, without any obstacles, using the Euclidean distance heuristic for a single trailer system. We found that this took a long time, about the order of tens of seconds. But, in comparison with Reeds Shepp distance metric, it took only about a few seconds to generate a path. 
Next, we implemented RRT to generate the tree and used A-star to generate the local path between nodes of the tree. This as well took significant time, and proved infeasible, as a result Reeds Shepp distance metric was used in finding the nearest node in RRT and hence incorporating the same in RRT tree building
Various bags of tricks such as, sampling end goals, steering and generating multiple nodes based on single sampled point, sampling randomly from a gaussian distribution whose mean is center of any one of the obstacles.
Currently the bag of tricks and hybrid RRT – local A star, is being debugged.

Challenges Faced and solutions found:
Due to higher dimensionality of truck-trailer system and limited computational resources, memory usage has to be optimised. To solve this problem we dynamically generated the graphs
Euclidean distance as metric to compute heuristic doesn’t generate feasible paths as per the non-holonomic constraints of the trailer truck system. Reeds Shepp distance was used as the metric. In this the turning radius was modified as larger than the truck to generate smoother trajectories. And this metric was updated to include the penalty of trailer’s angle not to be different from the angle of the truck.
The truck sometimes used to get stuck at locations close to the obstacles and can’t find path quickly. To solve this problem, in the local A-star, the heuristic was updated to include a penalty on distance from the obstacles of the form (1/(1+ exp(d)) where d is the distance of the truck from the centre of the obstacle. In the RRT portion, with a random probability, nodes are sample from a gaussian distribution, whose mean is the centre of a randomly chosen obstacle. This improves the chances of finding a path quickly, and is still complete in probability.
RRT would expand in random directions with no aim till goal is reached, which again increased the time to reach the goal. This was solved by sampling the end goal as the node, with a small probability of 0.01. And steering technique was used to generate multiple nodes in the direction of the sampled node from the nearest node. Note that the direction is determined by A –star which does assume that there are obstacles and the nearest node is determined by the Reeds Shepp distance metric.

Bag of Tricks:
Reedshepp distance matric was used both in heuristric of Astar and RRT(to find nearest node from the sample).
Given a nearest node and sample node a path was generated from nearest node to sampled node using Astar assuming no obstacles. All collision free points were addded to the tree. this increase rate of expansion of the tree
Goal biasing is present which samples the goal node with small probability
with a small probability points from a gaussian distribution around the goal, samples are taken.
If the alogrithm is stuck in a loop where it will always sample the same invalid node as the nearest node to the goal node. it will be flagged and removed from nearest neighbour node to goal node to further the algorithm.
