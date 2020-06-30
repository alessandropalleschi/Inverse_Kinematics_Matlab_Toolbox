robot_name = 'WRAPP_up_right_arm';
load(robot_name);
close all

q0 = robot.homeConfiguration;
% q0 = q_0_right(2:7).';
% tool = 'ee_link';
P = [  [1.1995+0.1;   1.2710;   0.8580]];
T0 = getTransform(robot.model,q0,robot.tool);
quat = rotm2quat(T0(1:3,1:3)*rotz(-pi/2));



T = generateTrajectories(1000,q0,robot.model,robot.tool,P,quat);
% T = repmat([P;quat.'],[1,200]);
qout_RP = RP_inverse(T,0.01,q0,robot.model,robot.limits.position,robot.tool);
% qout_RP = NB_inverse(T,0.001,q0,robot.model,robot.limits.position,robot.tool);
% qout_RP = SoT_Inverse(T,0.001,q0,robot.model,robot.limits.position,robot.tool);

qout_CLIK = Standard_IK(T,0.01,q0,robot.model,robot.tool);

% figure()
% plot(((qout_RP-med')./(limits.max-med)')');


figure()
hold on
plot(qout_RP')
plot(qout_CLIK')
hold off

figure()
show(robot.model,qout_RP(:,end)')
hold on
show(robot.model,qout_CLIK(:,end)')
hold off


qopt = optimize_one_arm(qout_RP,robot);

figure()
hold on
plot(0:0.1:length(qopt)*0.1-0.1,qopt')

% plot(qout_CLIK')
hold off
