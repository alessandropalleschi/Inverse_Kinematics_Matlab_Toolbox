function [q_out] = RP_inverse_dual_arm(x,xd,Ts,q_0_right,robot,limits,ee)

    
%task definition
jmax = limits.max;
jmin = limits.min;


N = 3;
DPI_lambda_max = 0.0001*10^4; 	% damping for pinv
DPI_epsilon = 0.0001;          % bound for pinv

beta_pos = 5/180*pi;
beta_vel = 0.1;

lambda = 0.9;    

kp = 5;
K = [1,1,kp];  	
%% whole parameters vector
param_vect = [DPI_lambda_max, DPI_epsilon, beta_pos, beta_vel, lambda, K];

%% compute using direct kinematics     
% DH_table_num = Franka_DH_table(q_0_right);
% [Tee_home] = Direct_Kin(DH_table_num);
% T_b_DH0 = eye(4);
% T_DH7_ee = eye(4);


% for i = 1:size(T.right,2)
%        
% %     ZYX(i,:) = rotm2eul(x_or_ee_des(:, :, i),'ZYX');
%         
% end

if(size(x.left,2) > size(x.right,2))
    x.right = [x.right repmat(x.right(:,end),1,size(x.left,2)-size(x.right,2))];
    xd.right = [xd.right repmat(xd.right(:,end),1,size(xd.left,2)-size(xd.right,2))];

elseif (size(x.left,2) < size(x.right,2))
    x.left = [x.left repmat(x.left(:,end),1,size(x.right,2)-size(x.left,2))];
    xd.left = [xd.left repmat(xd.left(:,end),1,size(xd.right,2)-size(xd.left,2))];

end
x_or_ee_des_left = zeros(3, 3, size(x.left,2));
traj_left = zeros(3,size(x.left,2));
x_or_ee_des_right = zeros(3, 3, size(x.right,2));
traj_right = zeros(3,size(x.right,2));

 iter_num_1 = size(x.left,2);
 x_des = cell(N, iter_num_1);  % init for speed
        for k = 1 : iter_num_1
            
                x_or_ee_des_left(:, :, k) = quat2rotm(x.left(4:end,k)');
    traj_left(:,k) = x.left(1:3,k);
    xd_left(:,k) = xd.left(:,k);
    x_or_ee_des_right(:, :, k) = quat2rotm(x.right(4:end,k)');
    traj_right(:,k) = x.right(1:3,k);
    xd_right(:,k) = xd.right(:,k);

            traj_des = {traj_left(:,k); x_or_ee_des_left(:,:,k);traj_right(:,k); x_or_ee_des_right(:,:,k)};
            xd_des_traj = [xd_left(:,k);xd_right(:,k)];
%             traj_des_right = {traj_right(:,k); x_or_ee_des_right(:,:,k)};

%             x_des(1,k) = jmax.';
%             x_des(2,k) = jmin.';
%             x_des(N-1,k) = {traj_des_right};
%             x_des(N,k) = {traj_des_left};
            x_des(:,k) = {jmax.'; jmin.'; traj_des};
            xd_des(:,k) = {0*jmax.'; 0*jmin.'; xd_des_traj};

        end        
        
        
        
        % variables for RP algorithm
    
    % flag showing if p is a task or a constraint.
    %0 task, 
    %1 max constraint,
    %2 min constraint 
    unil_constr = [1, 2, 0];
    
    % constraint value (NaN when not present)
    x_cons = { jmax jmin NaN};
 disp('Here');
%% algorithm

% execute algorithm
[q_out] = reverse_priority_pos_or_dual_arm( 	N, Ts, iter_num_1, ...
                                                        q_0_right, x_des,xd_des, ...
                                                        unil_constr, x_cons, param_vect,robot,ee);
