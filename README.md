# AerialRobotics

This repo includes matlab code for:

- Path planning algorithms (Dijkstra,  Astar,  Jump point search)
- Safe Flight Corridors
- Trajectory planning
- Quadrotor PD controller



## Required

MATLAB(R2019b is tested)



## Pic



### Path Planning

- #### dijkstra

  <img src="gifs/dijkstra.gif" alt="dijkstra" width="500">    dijkstra time is 1.642271 seconds.

- #### Astar

  <img src="gifs/Astar.gif" alt="Astar" width="500">      Astar time is :  0.755504 seconds.

- ####  Jump Point Search

​       <img src="gifs/JPS.gif" alt="JPS" width="500">            JPS time is :  0.451664 seconds.

### Safe Flight Corridors

- ####  find_ellipsoid

  <img src="imgs/ellipsoid-1.png" alt="polyhedron-1" width="400"><img src="imgs/ellipsoid-2.png" alt="polyhedron-2" width="400">

- #### find_polyhedron

<img src="gifs/polyhedron-1.gif" alt="polyhedron-1" width="400"><img src="gifs/polyhedron-2.gif" alt="polyhedron-2" width="400">



### Trajectory planning

Compare different Trajectory planning：

- #### Trajectory 1: use ['close form'](https://github.com/yrlu/quadrotor) get Trajectory planning.

<img src="gifs/closeForm.gif" alt="closeForm" width="270"><img src="imgs/close_postion.jpg" alt="closeForm" width="270"><img src="imgs/close_velocity.jpg" alt="closeForm" width="270"> 

The Trajectory pass every Path point.

- #### Trajectory 2: use 'Quadratic Programming' get Trajectory planning. use ['corridor constraints'](https://github.com/symao/minimum_snap_trajectory_generation) make  Ax< b.

<img src="gifs/corridorConstraints.gif" alt="corridorConstraints" width="270"><img src="imgs/corridorConstraints_postion.jpg" alt="corridorConstraints" width="270"><img src="imgs/corridorConstraints_velocity.jpg" alt="corridorConstraints" width="270"> 

The Trajectory don't need to pass every Path point.

minSnapValue(X + Y + Z) is : 9376.0901



- #### Trajectory 3: use 'Quadratic Programming' get Trajectory planning. use [SFC](https://github.com/LenaShengzhen/AerialRobotics) make Ax < b.

<img src="gifs/SFC.gif" alt="SFC" width="270"><img src="imgs/SFC_postion.jpg" alt="SFC" width="270"><img src="imgs/SFC_velocity.jpg" alt="SFC" width="270"> 

The Trajectory don't need to pass every Path point.

minSnapValue(X + Y + Z) is : 2958.5877



### Trajectory tracking

Trajectory tracking by use PD Controller, you can learn by [coursera](https://www.coursera.org/learn/robotics-flight/home/welcome).







## Reference

##### Paper:

[1] D. Harabor and A. Grastien. 2011. "Online Graph Pruning for Pathfinding on Grid Maps". In Proceedings of the 25th National Conference on Artificial Intelligence (AAAI), San Francisco, USA.

[2] S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor, et al., "Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments", IEEE Robotics and Automation Let- ters, vol. 2, no. 3, pp. 1688-1695, July 2017.





[3]D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors", inProc. 2011 IEEE Int. Conf. Robot.Autom.,2011



##### Code:

https://github.com/sikang/DecompUtil

https://github.com/symao/minimum_snap_trajectory_generation

https://github.com/yrlu/quadrotor

