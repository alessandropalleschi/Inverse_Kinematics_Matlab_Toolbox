clear
robot_name = 'AlterEgo';
load(robot_name);
close all

q0 = robot.homeConfiguration;
% q0 = q_0_right(2:7).';
% tool = 'ee_link';
Pright = [  [0.0382;  -0.2591;   0.3217]];
T0right = getTransform(robot.model,q0,robot.tool.right);
quatright = rotm2quat(T0right(1:3,1:3));




T0left = getTransform(robot.model,q0,robot.tool.left);
quatleft = rotm2quat(T0left(1:3,1:3));
Pleft = [  [0.0382;  0.2591;   0.3217]];
% T.left = repmat([T0(1:3,4);quat.'],[1,400]);
[x,xd] = dualArmgenerateTrajectories(robot.model,q0,robot.tool.right,robot.tool.left,100,100,Pright,quatright,Pleft,quatleft);
% T = repmat([P;quat.'],[1,200]);
x.right = x.right(:,end);
xd.right = xd.right(:,end);
x.left = x.left(:,end);
xd.left = xd.left(:,end);


qout_RP = RP_inverse_dual_arm(x,xd,0.01,q0,robot.model,robot.limits.position,robot.tool);
% tic
% s = compute_s_joint(qout_RP);
% time_step = 0.01;
% [qopt,topt] = optimize_one_arm(qout_RP,s,time_step,robot);
% 
% toc
figure()
show(robot.model,qout_RP(:,end)')
% light
hold on
% show(robot.model,qout_CLIK(:,end)')
hold off
