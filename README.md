# Phase-3
## A*-algorithm implementation with Differential constainsts of the turtlebot
### Description
In this project, A* algorithm is implemented considering the differential contraints of the turtle bot. We are considering the rigid robot to be the turtlebot. The action set of the rigid robot is determined using the left and right wheel velocities which are provided as inputs by the user. In addition to that, constaints are added to the action set by determining the equations of the motion of the turtlebot.
### Obstacle space
![Obstacle space](images/pic2.PNG)
### Actions
The actions of the robot are determined by left and right wheel velocities given by the user.

### Dependencies 
1. python -version 3
2. pygame


### Libraries used
1. Numpy
2. collections 
3. queue
4. math

### Run Code
Enter the following to run the code.

```
cd [to 'Code' directory]
python3 main.py
```

### Input Instruction:
As soon as you run the program, the following prompt occurs in the command window:
```
Robot considered is Turtlebot 2:
Enter cleareance
0.5
Enter start location s1 between -5 and 5 - (X-coordinate of start node)
-4
Enter start location s2 between -5 and 5 - (Y-coordinate of start node)
4
Enter the angle of the robot in degrees - (intial orientation of the robot)
0
Enter goal location g1 between -5 and 5 - (X-coordinate of goal node)
0
Enter goal location g2 between -5 and 5 - (Y-coordinate of goal node)
-3
```

### Sample output for rigid robot:
After running the python file
```
Cost took to reach the goal is: 395.64675298172733
Backtracking...
Total time taken 128.9800910949707
```

### Sample Output Video

# Phase-4:
In phase 4, we implement A* algorithm on turtlebot by simulating it on gazebo using ROS.

### Gazebo world:
The turtlebot obstacle space in gazebo is given below:
![Gazebo World](images/gazeboworld.png)

### Dependencies 
1. python -version 2.7
2. python -version 3
3. Gazebo
4. ROS

### Run the code:
First run the A* star code. For this python-3 version is required. Note the user must be in Differential-Drive-PP directory while running the code.
```
cd Differential-Drive-PP
python3 code/main.py
```
Now change the python version to python 2.7
```
cd Differential-Drive-PP/catkin_ws
catkin_make
source devel/setup.bash
cd ..
rosrun turtlebot_astar run.sh
```

Now open another terminal and run the following code.
```
cd Differential-Drive-PP
source catkin_ws/devel/setup.bash 
rosrun turtlebot_astar talker.py 
```

