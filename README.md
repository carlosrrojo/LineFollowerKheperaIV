# LineFollowerKheperaIV
Controller module for KheperaIV so it follows a line learnt using Q-learning implementation.

This controller was created for the robot [KheperaIV](https://www.cyberbotics.com/doc/guide/khepera4?version=cyberbotics:R2019a). 

The following module contains two behaviours implemented.

- Obstacle detection and avoidance.
- Line following

### Line Following

For the Q-learning algorithm we will create a action-state matrix with the following states and actions:

#### States

- S1: The robot leaves the line through the left side.
- S2: The robot leaves the line through the right side.
- S3: Any other case.

We consider the robot is leaving the line through the left side when the
leftmost infrared sensor doesn't sense the line and the rightmost infrared sensor is still on the line. And viceversa for the right side.

#### Actions

- A1: turn right
- A2: turn left
- A3: go straight

