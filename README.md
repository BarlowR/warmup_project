# Comp Robo Warmup Project
### Robert Barlow
### 9/29/20

## Introduction

As the first project for the course, Introduction to Computational Robotics, I've written 5 behaviors for a Neato robot in ROS/Python. I don't have access to a physical Neato, so the robot is simulated using the Gazebo environment. Below you will find a description of each behavior, a brief explaination of its implementation, and a short clip of the neato performing that behavior.


## Teleop
The first behavior on my list is a simple tele-op program for the neato to drive it around the simulated environement using a gamepad. the y axis of the joystick controls the forward velocity of the robot, and the x axis controls the angular turning velovity of the robot. 

#### Implementation
This is implmented using the inputs python library to capture the gamepad joystick and buttons, and a simple publisher to the Neato /cmd_vel topic to command the neato. Unfortunately the inputs library is blocking, so I'm leveraging a secondary thread to run the gamepad checker while the neato is commanded from the main thread.


## Point to Point
The next behavior is a simple navigation system; you pass a list of (x,y) coordinate tuples in, and the neato will drive to each point on the list in order, stopping at the last. The original requirement was to drive the neato in a 2x2 square, but I expanded out on the task a bit. 


#### Implementation
I've implemented this as a finite state machine, with each state cooreponding to the movement towards a given point. I use the Neato's Odometry topic to determine where the robot is in the simulation, and then compute the distance to the next point and the angle that the neato has to turn. To minimize driving in the wrong direction, I calculate the forward velocity as proportional to the next point but inversely proportional to the angle between the robot's current heading and the desired heading (this makes a nice sweeping path that can be seen in the below gif). Once the neato is within an absolute distance from the desired point, the state increments and the neato begins movement towards the next point.

<img src="https://github.com/BarlowR/warmup_project/blob/master/gotopoints.gif" width="960" height="540" />

## Wall Following
This behavior is a robust wall following system. It can follow a very roughly wall, and is robust to fast movement and sharp corners that can be both convex or concave.

#### Implementation
To follow a wall, I first create a 180 element array of weights, each corresponding to the cosine of the angle of each index, starting at 90 (I.E. element 90 is 1, element 0 & 180 are 0). Then, at each timestep, I compute the difference of each laser scan pair off of the left of the neato (the measurement at 91°- the measurement at 89°, then m(92°) - m(88°)) and multiply each difference by its cooresponding weight in the weight array. I sum all of these values, and from this I can tell if the neato needs to turn right or left to stay parallel with the wall. Next, I average each pair and once again multiply the result by the weight array and sum the result. Because the weight array is based off of the cosine function, this gives me the average distance to all of the readings on the left projected onto the neato's x axis. From this, I negate the a set distance from the wall, which tells me if the neato needs to turn closer to the wall or further away. I finally take the sum of these two measurements, and command the neato to turn accordingly. The forward velocity is fixed at 0.8. 

<img src="https://github.com/BarlowR/warmup_project/blob/master/wallfollow.gif" width="960" height="540" />

## Person Following
The person following behavior will search for the largest continuous object near the neato, and the neato will then go towards it until it is .5m away from the object. It will follow the object until a larger or closer object enters the laser range, at which point it will go follow that object.

#### Implementation
For this behavior, I wrote a recursive function that takes the neato laser scan and identifies the longest continuous set of ranges that are within a given threshold of each other. This function returns the startpoint and endpoint of this set of measurements, from which I then calculate the center and average distance to each point within the set. Then I simply use a proportional controller to turn the neato towards the object and drive .5m within the object. The function is continually called, so if a larger object is identified, the neato will turn and drive towards it. 

<img src="https://github.com/BarlowR/warmup_project/blob/master/personfollow.gif" width="960" height="540" />

## Obstacle Avoidance
The obstacle avoidance behavior does exactly what one would think- the neato drives forward and avoids obstacles. The neato changes its speed based on how constrained the environment infront of it is. 

#### Implementation
This was fairly simple to implement; I just took the weight array from the wall following behavior and took the element-wise product of it with the laser ranges from the front half of the neato. I then sum the values on the left half, and compare it to the same sum of values from the right half, which informs the direction that the neato turns. Finally, I take the whole front 180 degree weighted range measurements and set the velocity of the neato to be inversely proportional to this sum.  

<img src="https://github.com/BarlowR/warmup_project/blob/master/obstacleavoidance.gif" width="960" height="540" />

## Bonus: Obstacle Avoidance & Teleop

This is a teleop behavior that will do obstacle avoidance if you press the A button on the gamepad.

#### Implementation
I make an instance of the teleop class and an instance of the obstacle avoidance class and switch between them based on whether the button is pressed.

<img src="https://github.com/BarlowR/warmup_project/blob/master/stateMachine.gif" width="960" height="540" />


