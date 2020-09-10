# Manipulator-IK-Toolbox
Matlab Implementation of IK Algorithm for Serial Manipulators

## Requirements:
Matlab 2020a with Robotic System Toolbox

## Current Robots:

  - Kuka LWR IV+ without end-effector
  - Kuka LWR IV+ equipped with the Pisa/IIT SoftHand as end-effector
  - Franka Emika Panda without end-effector
  - Franka Emika Panda equipped with the Pisa/IIT SoftHand as end-effector
  - UR10 without end-effector
  - UR10 with SoftHand
  - WRAPP-up robotic platform for logistics

## Current IK Algorithms:
 - CLIK algorithm (position and orientation task)
 - Reverse Priority algorithm (position and orientation task and constraints on the joint limits)
 
 ## TO DO:
 - [ ] Include null-based IK algorithm
 - [x] Include UR10 equipped with the Pisa/IIT SoftHand as end-effector
 - [ ] Generalize the Reverse Priority algorithm to include different constraints
