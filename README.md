tangentbug
==========

Simulation of the tangent bug algorithm for robot navigation in ROS using Stage

### To run the example simulation in ROS Hydro:
- clone the repository into a new package called "tbug"
- run "roslaunch tbug tbug.launch
- use the bracket keys ([ and ]) to speed up or slow down the simulation (robots are very slow at default speeds)

#Quick Tips:
### To set new robot goals:
- go to the src folder of the package
- modify the goal0.py, goal1.py, and goal2.py nodes to publish different goals to the robot

### To use your own custom map for navigation:
- upload a map .png file to the world folder
- change the 16th line of tbug_demo.world to the name of the new .png file you uploaded

### To change the number of robots in simulation:
- go to the world folder and open tbug_demo.world
- open tbug_demo.world
- go to the bottom of the file where the 3 robots are declared. Add or remove robots as wanted.
- If robots were removed, modify the launch file so that it does not launch the goal, frames, and tangentbug nodes for that robot
- If robots were added, create new goal, frames, and tangentbug nodes for each of new robots and run those nodes during simulation
