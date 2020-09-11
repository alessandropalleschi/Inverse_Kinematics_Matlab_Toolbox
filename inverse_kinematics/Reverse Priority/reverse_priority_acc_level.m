function [q] = reverse_priority_acc_level(N, Ts, iter_num, q_0,J_0,qd_0, x_des,xd_des,xdd_des, unil_constr, x_cons, param_vect,robot, ee)
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
qd(:,1) = qd_0;
qd_prev = qd_0;
qdd_prev = zeros(size(q_0,2),1);
Jt{1} = J_0; 
% ---------------------------------------------------------------------
% specific part
J{1} = eye(size(q_0,2));
J{2} = eye(size(q_0,2));
dJ{1} = zeros(size(q_0,2));
dJ{2} = zeros(size(q_0,2));
J{3} = eye(size(q_0,2));
J{4} = eye(size(q_0,2));
dJ{3} = zeros(size(q_0,2));
dJ{4} = zeros(size(q_0,2));

xd_des_prev{N} = zeros(6,1); 
% ---------------------------------------------------------------------
% k = 2;
ep = 1;
eo = 1;
k = 2;
while(norm(ep) >=1.0*1e-3 || norm(eo)>=1e-1)    % specific part
        
        %     qd1 = qd(1, end);
        %     qd2 = qd(2, end);
        %     qd3 = qd(3, end);
        %     qd4 = qd(4, end);
        %     qd5 = qd(5, end);
        %     qd6 = qd(6, end);
        %     qd7 = qd(7, end);
        
        % numeric jacobian
        Jorp = geometricJacobian(robot,q(:,k-1)',ee);
        J{N} = [Jorp(4:6,:);Jorp(1:3,:)];
        T = getTransform(robot,q(:,k-1)',ee);
        
        dJ{N} = (Jt{k-1}-J{N})/Ts;
        Jt{k} = J{N};
        % actual x
        x(:,k) = {qd(:,k-1); qd(:,k-1);q(:,k-1); q(:,k-1);{T(1:3,4);T(1:3,1:3)}};
        
        x_cur = x(:,k);

        
        % -----------------------------------------------------------------
        if(k<=iter_num)
        
        x_des_cur = x_des(:,k);
        xd_des_cur = xd_des(:,k);
        xdd_des_cur = xd_des(:,k);
%         x_des_prev = x_des(:,k-1);
%         xd_des_prev = xd_des(:,k-1);
%         xdd_des_prev = xdd_des(:,k-1);
        
        else
        
        x_des_cur = x_des(:,iter_num);
        xd_des_cur = xd_des(:,iter_num);
        xd_des_cur{N} = 0*xd_des_cur{N};
        xdd_des_cur{N} = 0*xdd_des_cur{N};
%         x_des_prev = x_des(:,iter_num);
%         xd_des_prev = xd_des(:,iter_num);
%         xdd_des_prev = xdd_des(:,iter_num);
        end
        for p = N:-1:1     % e.g.: 3 2 1
            
            % error computation and xd computation
            if unil_constr(p)
                
            else
                % compute orientation error, using quaternions
                xfin = x_des(:,end);
                eo = eo_using_quat(x_cur{p}{2}, xfin{p}{2});
                ep = x_cur{p}{1} - xfin{p}{1};
                e = -[ep;eo];
            end
            
            norm(ep)
            
        end
        % x: cell array
        
        
        if k <= length(x_cons(:,1))
            x_cons_cur = x_cons(k,:);       % select the k-th step row
        else
            x_cons_cur = x_cons(end,:);     % works also for time-invariant
        end
        
        % compute tasks desired vel (for trajectory tracking)
%         for p = N:-1:1
%             
%             % if x_des is a matrix (R), xd_des_cur will be an ang. vel. w:
%             % it's a matrix (R)
%             if(unil_constr(p))
%             else
%                 dR = (x_des_cur{p}{2} - x_des_prev{p}{2}) / Ts;    % Rdot
%                 w_ee_hat = dR*x_des_cur{p}{2}.';             % Rdot * R.'
%                 w_ee = skew_2_vect(w_ee_hat);
%                 
%                 
%                 xd_des_cur{p} = [(x_des_cur{p}{1} - x_des_prev{p}{1}) / Ts;w_ee];
%                 xdd_des_cur{p} = (xd_des_cur{p}-xd_des_prev{p})/Ts;
%                 xd_des_prev{p} = xd_des_cur{p};
%             end
%         end
%         
        
        
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
            [qdd_des,qd_des] = reverse_priority_step_acc_level(N, qd_prev,qdd_prev, x_des_cur, ...
                xd_des_cur,xdd_des_cur, unil_constr, ...
                x_cons_cur, param_vect, ...
                J,dJ, x_cur,Ts);
            
            qdd_prev = qdd_des;
            %     toc
            %     user message
            %             disp('... done');
            
            q(:,k) = q(:,k-1) + qd_prev*Ts+qdd_des*Ts^2/2;
            qd(:,k) = qd_prev+qdd_des*Ts;
            qdd(:,k) = qdd_des;

            qd_prev = qd_prev+qdd_des*Ts;
            k = k+1;
    % append to vectors
    %             e = [e, e_new];
    %n = norm(qd_des)./norm(qd_com);
end
% user message
disp('Reverse priority algorithm complete');

end