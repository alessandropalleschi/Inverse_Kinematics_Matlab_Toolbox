function [q_out] = RP_inverse(T,Ts,q_0_right,robot,limits,ee)

    
%task definition
jmax = limits.max;
jmin = limits.min;


unil_constr = [1*ones(1,length(jmax)) 2*ones(1,length(jmin)) 0];
%%
%% parameters for reverse priority algorithm
N = 2*length(jmax)+1;
DPI_lambda_max = 0.01*10^4; 	% damping for pinv
DPI_epsilon = 0.1;          % bound for pinv

beta_pos = 0.01;
beta_vel = 0.1;

lambda = 0.9;    

kp = 50;
K = [ones(1,N-1), kp];  	% error gain vector

%% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];

%% compute using direct kinematics     
% DH_table_num = Franka_DH_table(q_0_right);
% [Tee_home] = Direct_Kin(DH_table_num);
% T_b_DH0 = eye(4);
% T_DH7_ee = eye(4);
x_or_ee_des = zeros(3, 3, size(T,2));
traj = zeros(3,size(T,2));
for i = 1:size(T,2)
       
    x_or_ee_des(:, :, i) = quat2rotm(T(4:end,i)');
    traj(:,i) = T(1:3,i);
%     ZYX(i,:) = rotm2eul(x_or_ee_des(:, :, i),'ZYX');
        
end
 iter_num_1 = size(traj,2);
        
        x_des = cell(N, iter_num_1);  % init for speed
        for k = 1 : iter_num_1
            traj_des = {traj(:,k); x_or_ee_des(:,:,k)};
            x_des(1:length(jmax),k) = num2cell(jmax.');
            x_des(length(jmax)+1:length(jmax)+length(jmin),k) = num2cell(jmin.');
            x_des(N,k) = {traj_des};
        end        
        
        
        
        % variables for RP algorithm
    
    % flag showing if p is a task or a constraint.
    %0 task, 
    %1 max constraint,
    %2 min constraint 
%     unil_constr = [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 0];
    
    % constraint value (NaN when not present)
    x_cons = [  jmax jmin NaN];
 disp('Here');
%% algorithm

% execute algorithm
[q_out] = reverse_priority_pos_or_7j( 	N, Ts, iter_num_1, ...
                                                        q_0_right, x_des, ...
                                                        unil_constr, x_cons, param_vect,robot,ee);
