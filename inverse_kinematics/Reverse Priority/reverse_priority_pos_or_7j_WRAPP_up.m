function [q] = reverse_priority_pos_or_7j_WRAPP_up(N, Ts, iter_num, q_0, x_des, unil_constr, x_cons, param_vect,robot, ee)
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
q(:,1) = q_0';
qd_prev = zeros(size(q_0,2),1);

% ---------------------------------------------------------------------
% specific part
J{1} = eye(size(q_0,2));
J{2} = eye(size(q_0,2));

% ---------------------------------------------------------------------
% k = 2;
for k=2:iter_num
    % specific part
    
        %     qd1 = qd(1, end);
        %     qd2 = qd(2, end);
        %     qd3 = qd(3, end);
        %     qd4 = qd(4, end);
        %     qd5 = qd(5, end);
        %     qd6 = qd(6, end);
        %     qd7 = qd(7, end);
        
        % numeric jacobian
        JorpL = geometricJacobian(robot,q(:,k-1)',ee.left);
        JorpR = geometricJacobian(robot,q(:,k-1)',ee.right);
        J{N-1} = [JorpR(4:6,:);JorpR(1:3,:)];

        J{N} = [JorpL(4:6,:);JorpL(1:3,:)];
        TL = getTransform(robot,q(:,k-1)',ee.left);
        TR = getTransform(robot,q(:,k-1)',ee.right);
        
        % actual x
        x(:,k) = {q(:,k-1); q(:,k-1);{TR(1:3,4);TR(1:3,1:3)};{TL(1:3,4);TL(1:3,1:3)}};
        
        
        
        % -----------------------------------------------------------------
        
        x_cur = x(:,k);
        
        x_des_cur = x_des(:,k);
        x_des_prev = x_des(:,k-1);
        
%         for p = N:-1:1     % e.g.: 3 2 1
%             
%             % error computation and xd computation
%             if unil_constr(p)
%                 
%             else
%                 % compute orientation error, using quaternions
% %                 e_o = eo_using_quat(x_cur{p}{2}, x_des_cur{p}{2});
% %                 e_p = x_cur{p}{1} - x_des_cur{p}{1};
% %                 e = -[e_p;e_o];
%             end
            
            
            
%         end
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
            [qd_des] = reverse_priority_step_limits(N, qd_prev, x_des_cur, ...
                xd_des_cur, unil_constr, ...
                x_cons_cur, param_vect, ...
                J, x_cur);
            qd_prev = qd_des;
            
            %     toc
            %     user message
            %             disp('... done');
            
            q(:,k) = q(:,k-1) + qd_des*Ts;
    % append to vectors
    %             e = [e, e_new];
    %n = norm(qd_des)./norm(qd_com);
end
% user message
disp('Reverse priority algorithm complete');

end