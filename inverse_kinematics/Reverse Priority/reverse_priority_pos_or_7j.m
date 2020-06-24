function [q] = reverse_priority_pos_or_7j(N, Ts, iter_num, q_0, x_des, unil_constr, x_cons, param_vect,robot, ee)
%{
===========================================================================
	Exactly as the general reverse_priority.m, but enhanced in performance.
    On the other hand, this can be used only when all the following tasks
    are present:

    - all 7 joints limits constraints
    - position and orientation task

===========================================================================
%}

% user message
disp('Reverse priority algorithm initialization');
% initialization (i.e. k = 1)
q(:,1) = q_0;
qd_prev = zeros((N-1)/2,1);

% ---------------------------------------------------------------------
% specific part
I = [eye((N-1)/2);eye((N-1)/2)];

for i = 1:1:N-1
    J{i} = I(i,:);
end
% ---------------------------------------------------------------------
k = 2;
while(k <= iter_num)

    % specific part
    
    
    %     qd1 = qd(1, end);
    %     qd2 = qd(2, end);
    %     qd3 = qd(3, end);
    %     qd4 = qd(4, end);
    %     qd5 = qd(5, end);
    %     qd6 = qd(6, end);
    %     qd7 = qd(7, end);
    
    % numeric jacobian
    Jorp = geometricJacobian(robot,q(:,end)',ee);
    J{N} = [Jorp(4:6,:);Jorp(1:3,:)];
    T = getTransform(robot,q(:,end)',ee);

    % actual x
    x{N,k} = {T(1:3,4);T(1:3,1:3)};
    
    x(1:length(q(:,end)),k) = num2cell(q(:,end));
    x(length(q(:,end))+1:2*length(q(:,end)),k) = num2cell(q(:,end));
    
    
    % -----------------------------------------------------------------
    
    x_cur = x(:,k);
    
        x_des_cur = x_des(:,k);
    x_des_prev = x_des(:,k-1);

    for p = N:-1:1     % e.g.: 3 2 1
        
        % error computation and xd computation
        if unil_constr(p)
            
        else
            % compute orientation error, using quaternions
            e_o = eo_using_quat(x_cur{p}{2}, x_des_cur{p}{2});
            e_p = x_cur{p}{1} - x_des_cur{p}{1};
            e = [e_p;e_o];
        end
        
        
        
    end
    % x: cell array
    
    
    if k <= length(x_cons(:,1))
        x_cons_cur = x_cons(k,:);       % select the k-th step row
    else
        x_cons_cur = x_cons(end,:);     % works also for time-invariant
    end
    
    % compute tasks desired vel (for trajectory tracking)
    for p = N:-1:1
        
        % if x_des is a matrix (R), xd_des_cur will be an ang. vel. w:
        % it's a matrix (R)
        if(unil_constr(p))
        else
            dR = (x_des_cur{p}{2} - x_des_prev{p}{2}) / Ts;    % Rdot
            w_ee_hat = dR*x_des_cur{p}{2}.';             % Rdot * R.'
            w_ee = skew_2_vect(w_ee_hat);
            
            
            xd_des_cur{p} = [(x_des_cur{p}{1} - x_des_prev{p}{1}) / Ts;w_ee];
        end
    end
    
    
    
    % -----------------------------------------------------------------
    % specific part
    
    %     % (qdMAX) scale gain by position error
    %     param_vect2 = param_vect;
    %     if norm(e) > 0.1
    %        param_vect2(6:6+N-1) = 0.1 * param_vect(6:6+N-1);
    %     end
    
    % -----------------------------------------------------------------
    
    % user message
    %         disp('... starting computation');
    % n
    %     tic
    [qd_des] = reverse_priority_step(N, qd_prev, x_des_cur, ...
        xd_des_cur, unil_constr, ...
        x_cons_cur, param_vect, ...
        J, x_cur);
    qd_prev = qd_des;
    
    %     toc
    %     user message
    %             disp('... done');
    
    q = [q, q(:,end) + qd_des*Ts];
    % append to vectors
    %             e = [e, e_new];
    %n = norm(qd_des)./norm(qd_com);
    k = k+1;
end
% user message
disp('Reverse priority algorithm complete');

end