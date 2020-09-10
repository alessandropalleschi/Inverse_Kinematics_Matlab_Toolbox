# Inverse_Kinematics_Matlab_Toolbox
Matlab Implementation of Inverse Kinematics Algorithms

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
  - Abb Yumi
  - Alan mobile manipulator
  - AlterEgo wheeled humanoid Robot

## Current IK Algorithms:
 - Single Arm CLIK algorithm (position and orientation task)
 - Single Arm Reverse Priority algorithm (position and orientation task and constraints on the joint limits)
 - Dual Arm Reverse Priority algorithm (position and orientation task and constraints on the joint limits)
 
 ## TO DO:
 - [ ] Include null-based IK algorithm
 - [ ] Generalize the Reverse Priority algorithm to include different constraints and tasks
