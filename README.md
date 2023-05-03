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


Additional Information:
\begin{abstract}
One of the most challenging tasks for truck drivers is maneuvering the truck-trailer system in different parking scenarios. In this project, we aim to develop and test path planning algorithms for this application and verify via simulations of their performance as per different metrics. We believe this project has the potential to be applied in areas of autonomous driving or driving assist.
\end{abstract}


\section{Introduction}
Trailer parking is a crucial aspect of the transportation industry. The traditional methods of trailer parking are time-consuming and require human intervention. Autonomous vehicles have been a hot topic in recent years, with significant advancements in technology making them a more feasible option for everyday use. One of the challenges facing autonomous vehicles is parking, especially for large vehicles such as trailers. Trailer parking is a complex task that requires precise and accurate parking skills. With the increasing demand for autonomous vehicles, the development of autonomous trailer parking is becoming increasingly important. In this project, we propose to develop a path-planning system for autonomous trailer parking. This system will be capable of navigating a trailer through a yard or parking area and successfully parking it in a designated spot.

\section{Background}

One of the most challenging tasks for truck drivers is maneuvering the truck-trailer system in different parking scenarios. The problem of parking comes under the domain of planning with kinematic constraints or non-holonomic constraints. A non-holonomic constraint is a non-integrable constraint. That is, for this problem, they are of velocity constraint type. For the trailer truck model, the kinematic model is already prevalent in the literature as noted here \cite{b4}. The reason why this problem is hard to solve is because of the limited configurations the obstacles allow for the trailer truck to access, as a result, solutions should be of the form wherein the truck should go back and forth in the same place, with just tiny adjustments in its heading to create the possibility of going into the parking space. There have been some research papers, which describe novel algorithms to solve this problem such as \cite{b5}. Though there aren't many reports of stress testing these algorithms against other algorithms as well as extending this problem to the multi-trailer case.

\section{Methodology}
In this project, we will be building on top of a mathematical model of a 1-trailer system where there is one pulling truck and 1 trailer being pulled by it. The trailer is hinged at the center of the rear axle of the pulling truck. The Kinematic model for the 1-trailer system defined by \cite{b1} is used in this project (shown in figure 1). The model simplifies 4 wheels to 1 turning wheel at the center. As the speed of the trailer when parking will be low, this approximate assumption is very accurate. This model will first be extended to the 2-trailer system and then multi-trailer systems will be explored based on the result of the 2-trailer system.\\

For the path planning of the truck-trailer system, we will be using a number of path-planning algorithms such as hybrid A*, MPC-RRT, etc. to compare and provide an analysis of the effectiveness of different algorithms. The major challenge, which determines the efficacy of these algorithms is to take into account the non-holonomic constraints of the model along with the constraints of the map.\\

We will be simulating our model in a 2d grid in a known environment. The algorithm will know all the relevant information such as the location of other vehicles, parking spots, and obstacles. This information will be processed to generate a feasible path for the trailer. All the algorithms will be able to avoid obstacles and maneuver the environment with non-holonomic constraints in place.

\section{Project Status Update}

\textbf{Overview:}
We chose pygame to create a simulation environment. Initially, the Ackerman steering model was used to simulate the motion of a car. This was extended to one trailer system. The dynamics of the trailer-truck system was simulated using the acceleration model. Due to the complexity of higher dimensions, we decided to go with the constant acceleration model. In the current simulation the update rate is set at 10 Hz. And in the current state the trailer-truck is being operated using inputs from the keyboard. In the same environment rectangular static obstacles, were placed for sanity checks of the final environment.\\

\textbf{Challenges faced}: 
We extended our work post the timeline by us, to create a graph of the configuration space to apply algorithms like Djikstra or BFS, to set a baseline. But due to the 'curse of dimensionality' computational power and memory requirements for creating a complete graph of the 6-dimensional configuration space (x, y, velocity, trailer angle, trailer-truck angle, steering angle) is very high.\\

\textbf{Way Forward}: 
We are currently on track as per our timeline and we are exploring and learning about the algorithms which we can apply to our simulations.\\

\section{Goals}
\begin{enumerate}
    \item Develop a path-planning algorithm capable of handling multiple trailers in compact spaces
    \item Algorithm should consider the kinematics of the system
    \item Algorithm should take into account non-holonomic constraints while navigating and avoiding obstacles
    \item To evaluate the performance of the algorithm and demonstrate its feasibility, such as:
     \subitem (i) Time taken to generate the path  
     \subitem (ii) After the path is generated, the time taken to reach the end goal.
\end{enumerate}

\section{Timeline}

\begin{itemize}
   \item \sout{  February 17: Simulation setup to be ready along with trailer truck system, which has the kinematic model included.}
    \item March 10: Single trailer solution to be ready for at least one algorithm (Hybrid-A*).
    \item March 24: Multi-trailer system solution to be ready for at least one algorithm.
    \item April 07: Multiple algorithms to be stress tested (at least Hybrid A*, Cl-RRT\cite{b5}).
    \item April 21: Tested under dynamic/unknown obstacles and report/presentation to be developed for submission. 
\end{itemize}

\section{Expected Results}
The proposed path planning algorithm for autonomous trailer parking will provide a highly effective solution for the challenge of parking trailers in complex environments. The system will be designed to handle multiple trailers in compact spaces with difficult kinematics while considering non-holonomic constraints and obstacle avoidance. A graph of paths generated by the algorithm will be shown, along with the motion of trailers parking in a given goal location. A performance evaluation of different path planning algorithms implemented will be provided to understand the effectiveness of each algorithm.

Note: We achieved the desired results and goals

Results and future work:
Results :
.We have simulated a path from the Hybrid RRT – local A –star path planner. This appears to be a highly effective solution, being able to find paths for multiple long trailers. 
It is much more efficient than the standard solutions.
Finally results, quantitive comparisons and implementation code and video will be posted along with the final report.
Future work :
 Bi-directional RRT’s can be incorporated. 
Hybrid RRT – local A –star can be updated to include RRT* to generate low-cost trajectories
