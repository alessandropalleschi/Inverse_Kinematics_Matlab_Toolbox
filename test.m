robot_name = 'AlterEgo';
load(robot_name);
close all

q0 = robot.homeConfiguration;
% q0 = q_0_right(2:7).';
% tool = 'ee_link';
T0 = getTransform(robot.model,q0,robot.tool.right);
qfin = [0.4 0.2 0 0 0 0 0 pi/4 -pi/3 pi/2 -pi/18 0]
Tf = getTransform(robot.model,qfin,robot.tool.right);
quat = rotm2quat(Tf(1:3,1:3));
P = [  Tf(1:3,4)];


[x,xd] = generateTrajectories(100,q0,robot.model,robot.tool.right,P,quat);
% x = x(:,end);
% xd = xd(:,end);
qout_RP = RP_inverse_limits(x,xd,0.01,q0,robot.model,robot.limits.position,robot.tool.right);
figure()
show(robot.model,qout_RP(:,end)','Frames','off')
light
zlim([-inf inf])
% q_sim = timeseries(qopt.',topt);
q_sim = timeseries(qout_RP.',0:0.01:length(qout_RP)*0.01-0.01);
% hold on
% % show(robot.model,qout_CLIK(:,end)')
% hold off


