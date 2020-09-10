function [q_out] = RP_inverse_limits(x,xd,Ts,q_0_right,robot,limits,ee)

    
%task definition
jmax = limits.max;
jmin = limits.min;


unil_constr = [1, 2, 0];
%%
%% parameters for reverse priority algorithm
N = 3;
DPI_lambda_max = 0.01*10^4; 	% damping for pinv
DPI_epsilon = 0.0001;          % bound for pinv

beta_pos = 5/180*pi;
beta_vel = 0.01;

lambda = 0.9;    

kp = 10;
K = [1,1, kp];  	% error gain vector

%% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];

%% compute using direct kinematics     
% DH_table_num = Franka_DH_table(q_0_right);
% [Tee_home] = Direct_Kin(DH_table_num);
% T_b_DH0 = eye(4);
% T_DH7_ee = eye(4);
% x_or_ee_des = zeros(3, 3, size(x,2));
% traj = zeros(3,size(x,2));
% for i = 1:size(T,2)
%        
%     x_or_ee_des(:, :, i) = quat2rotm(T(4:end,i)');
%     traj(:,i) = T(1:3,i);
% %     ZYX(i,:) = rotm2eul(x_or_ee_des(:, :, i),'ZYX');
%         
% end
 iter_num_1 = size(x,2);
        
        x_des = cell(N, iter_num_1);  % init for speed
        for k = 1 : iter_num_1
            traj_des = {x(1:3,k); quat2rotm(x(4:end,k)')};
            x_des(:,k) = {jmax.'; jmin.'; traj_des};
            xd_des(:,k) = {0*jmax.'; 0*jmin.'; xd(:,k)};
        end        
        
        
        
        % variables for RP algorithm
    
    % flag showing if p is a task or a constraint.
    %0 task, 
    %1 max constraint,
    %2 min constraint 
%     unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0];
    
    % constraint value (NaN when not present)
    x_cons = {  jmax jmin NaN};
 disp('Here');
%% algorithm

% execute algorithm
[q_out] = reverse_priority_pos_or_limits( 	N, Ts, iter_num_1, ...
                                                        q_0_right, x_des,xd_des, ...
                                                        unil_constr, x_cons, param_vect,robot,ee);
