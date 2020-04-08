# Line Following and Obstacle Avoidance Lego Mindstorms Robot :oncoming_automobile:

LEGO Mindstorms robot set have been used in order to build a structure of a mobile robot for missions on following white line and avoiding obstacles sequentially. Variety of equipment have been used to complete this task as listed below:
-	3 Motor
-	1 Light sensor 
-	1 Ultrasonic sensor 

**Steps of Robot:**

1.	Find the line
2.	Follow the line until Ultrasonic sensor detects an obstacle under threshold
3.	Adjust robot direction via setting powers of motors, which are connected to the wheels, in order to stay parallel to the wall
4.	Rotate Ultrasonic sensor 90 degree left against the wall then keep distance constant by applying PID controller to be able to start obstacle avoidance mission
5.	When light sensor realizes the first white line, this intermediate section finishes by rotating ultrasonic sensor to zero position and next section starts
6.	Go forward till seeing another obstacle then circumnavigate the obstacle until robot direction becomes zero. Then keep moving straight. Use this cycle since there is nothing greater than the threshold value on the light sensor
7.	When light sensor recognizes the white line, finish the mission.

**Hardware of Robot**

To be able to complete the tasks mentioned above the robot structure must be rigid. Otherwise, even if the code is feasible to finish all tasks, we would have disturbances and problems because of the structure. Also, positions of sensors are as important as rigidity of the body. As an instance, the light sensor has limited range to detect differences on the ground. Thus, it should be very close to the ground. Another case is about ultrasonic sensor position. We kept it in front of the robot to have enough free space to rotate. It would not be an optimal feasible solution for other positions because of other parts of the robot as wheels and base frame.


**Software of Robot**

***Line Following Mission :***

In line following section PI controller have used which is useful for the mission. First of all, we had to determine Kp, Kd and Ki values, which are the position controller gains. As we know from lecture notes, Kp is to decrease the rise time. Ki is to eliminate the steady state error. Kd is to reduce the overshoot. We have applied Ziegler-Nicholds method to find best Kp for the system. Begin with a low/zero value of gain Kp, then, increase it until the robot tracks the line. In order to keep exactly on the edge of the line, Ki is increased. At last, in order to decrease the oscillations, Kd is increased before the robot misses the line.

In order to apply PI controller, the error should be defined. The error is the difference between the value read by light sensor and the mean value (offset value) of ground light level and line light level. The proportional part will make the robot turn to minimize the error. If the error is negative, the robot tries to rotate to find line. Else, the robot tries to rotate opposite direction to reach edge of line. In order to make the motion more effective, integral action is necessary to sum up the error calculated in each cycle.

***Wall Following :***

This part of the code is a controlled intermediate task which is written to reach obstacle detection area perpendicular as shown in following figure. During this period, the robot tries to keep its distance 30 cm far from the wall by PI control, while driving through obstacle area. This is the most important part for obstacle detection algorithm, because the algorithm in obstacle detection area requires global positioning as odometry. Thus, this task is essential to keep initial guess for odometry direction as more precise as possible.

***Obstacle Avoidance Mission :***

There are several algorithms to manage this mission of the project which is obstacle detection. The task is to reach the white line at the end without touching randomly placed boxes. In order to manage two switching algorithms have used, the first one that had been tried was seeking an object such that robot goes forward until ultrasonic sensor detects any obstacle in front of it with a distance of 20 cm, than it scans 100° (±50°) area with help of 3rd motor to choose which direction is the most available to go through. It was supposed to repeat that algorithm so on to until light sensor detects another white line.

Key element of this part is Odometry algorithm which provides us a knowledge of robot location on x, y axis, and direction in angle. Use of y and absolute direction angle information in two sections of the obstacle detection algorithm
