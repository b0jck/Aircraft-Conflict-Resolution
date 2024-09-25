# Aircraft-Conflict-Resolution
This work proposes a Hybrid system approach to simplified Aircraft Conflict Resolution problem. We consider a simplified scenario with only 2 airplanes (agents) with a planar dynamics. The hybrid systems are composed of two different dynamics: a straight trajectory and a semicircular path (of curvature radius R)
### • aircraft_simulation.m
Matlab simulation of the conflict resolution. The user is asked to specify initial orientation of the two aircrafts. The simulation then strategically places the aircrafts in a collision path and calculates all the margins for collision avoidance. The user can select the curvature radius R and the time to start the maneuver. At this point, a simple animation of the two planes is rendered. If the selected values are in the displayed safe range, the collision will be avoided, otherwise the user can watch on the animation the inevitable collision.

### • safe_plotter.m
Matlab static visualization of the safe sets of the relative dynamics of the first airplane with respect to the second.

### .pdf files
The other two files are the mathematical computations behind the Matlab simulation: in particular the backwrds integration of the dynamics for computing the safe sets.
