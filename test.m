robot_name = 'alan_hand';
load(robot_name);
close all

q0 = robot.homeConfiguration;
% q0 = q_0_right(2:7).';
% tool = 'ee_link';
P = [  [1.1995+0.3;  4.2710;   0.8580]];
T0 = getTransform(robot.model,q0,robot.tool);
quat = rotm2quat(rotz(130)*T0(1:3,1:3)*rotz(90));



[x,xd] = generateTrajectories(100,q0,robot.model,robot.tool,P,quat);
% T = repmat([P;quat.'],[1,200]);
% qout_RPacc = RP_inverse_acc(T,0.01,q0,robot.model,robot.limits.position,robot.tool);
% qout_RP = NB_inverse(T,0.001,q0,robot.model,robot.limits.position,robot.tool);
% qout_RP = SoT_Inverse(T,0.001,q0,robot.model,robot.limits.position,robot.tool);
qout_RP = RP_inverse_limits(x,xd,0.01,q0,robot.model,robot.limits.position,robot.tool);

% qout_CLIK = Standard_IK(T,0.01,q0,robot.model,robot.tool);

% figure()
% plot(((qout_RP-med')./(limits.max-med)')');

% tic
% s = compute_s_joint(qout_RP);
% time_step = 0.01;
% [qopt,topt] = optimize_one_arm(qout_RP,s,time_step,robot);
% 
% toc
% % plot(qout_CLIK')
% % hold off
% 
% q_sim = timeseries(qopt.',topt);
% 
% figure()
% hold on
% plot(q_sim')
% % plot(qout_CLIK')
% hold off
% 
figure()
show(robot.model,qout_RP(:,end)','Frames','off')
light
xlim([0 inf])
ylim([0 inf])
zlim([0 inf])
% q_sim = timeseries(qopt.',topt);
q_sim = timeseries(qout_RP.',0:0.01:length(qout_RP)*0.01-0.01);
% hold on
% % show(robot.model,qout_CLIK(:,end)')
% hold off
Tfin = [rotz(90)*T0(1:3,1:3)*rotz(90) [P];0 0 0 1];
Treal = getTransform(robot.model,qout_RP(:,end).',robot.tool);

