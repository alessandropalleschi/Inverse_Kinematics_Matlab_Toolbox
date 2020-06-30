function q = Standard_IK(Traj,Ts,q0,robot,ee)
%STANDARD_IK Summary of this function goes here
%   Detailed explanation goes here
if(isrow(q0))
    q0 = q0.';
end
DPI_lambda_max = 0.01*10^4; 	% damping for pinv
DPI_epsilon = 0.01;          % bound for pinv
K = 10*eye(6);

q = zeros(size(q0,1),size(Traj,2));
q(:,1) = q0;

for i=2:size(Traj,2)        
        
        Jorp = geometricJacobian(robot,q(:,i-1).',ee);
        J = [Jorp(4:6,:);Jorp(1:3,:)];
        T = getTransform(robot,q(:,i-1).',ee);
        
        e_o = eo_using_quat(T(1:3,1:3),quat2rotm(Traj(4:end,i)'));
        e_p = T(1:3,4) - Traj(1:3,i);
        e = -[e_p;e_o];
%         norm(e)

    dR = (quat2rotm(Traj(4:end,i)') - quat2rotm(Traj(4:end,i-1)')) / Ts;    % Rdot
    w_ee_hat = dR*quat2rotm(Traj(4:end,i)').';             % Rdot * R.'
    w_ee = skew_2_vect(w_ee_hat);     
    xd = [(Traj(1:3,i) - Traj(1:3,i-1)) / Ts;w_ee];

        J_pinv = damped_pseudo_inverse(J, ...
                DPI_lambda_max, ...
                DPI_epsilon);
            qdes = J_pinv*(xd+K*e);
            
            q(:,i) = q(:,i-1) + qdes*Ts;
%             qaux_old = qaux;
%     i
end

