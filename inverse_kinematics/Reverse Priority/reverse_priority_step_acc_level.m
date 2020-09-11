function [qdd_des,qd_des,h_lim] = reverse_priority_step_acc_level(N, qd_prev,qdd_prev, x_des_cur, xd_des_cur,xdd_des_cur, unil_constr, x_cons_cur, param_vect, J,dJ, x_cur,Ts)
%{
===========================================================================
	This function executes one single step of the reverse priority
    algorithm, which computes the joint velocities necessary
    for multiple tasks execution.
    The tasks, which can be unilateral or bilateral constraints, are
    specified with different priorities and processed in reverse order,
    starting from the lower priority task.

    The task execution error is also returned, for plots and analysis.

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    N               [1 x 1]   	tasks number

	qd_prev       [j_num x 1]  	vector with previous step joint velocity

    x_des_cur       {N x 1}   	cell array with current desired tasks
                                position

    xd_des_cur      {N x 1}    	cell array with current desired tasks
                                velocity

	unil_constr     [1 x N]     vector which elements are:
                                    0 if the corresponding x_des is a task
                                    1 if it's a max unilateral constraint
                                    2 if it's a min unilateral constraint

	x_cons_cur      [1 x N]     vector with the current limits for max and
                                min constraints, depending on unil_constr

	param_vect    [1 x (N+5)]   vector with algorithm parameters:

        DPI_lambda_max  [1 x 1]     damping for damped pseudo inverse

        DPI_epsilon     [1 x 1]   	bound for damped pseudo inverse

        beta_pos        [1 x 1]   	position buffer for un. constr.

        beta_vel        [1 x 1]    	velocity buffer for un. constr.

        lambda          [1 x 1]    	damping lambda for task N+1

        K               [1 x N]   	error gain vector

	J               {1 x N}   	cell array with jacobians associated with
                                the tasks

	x_cur           {1 x N}   	cell array with current tasks position

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	qd_new        [j_num x 1]   vector with new computed joint velocity
    
    e_new           {N x 1}   	cell array with new error

===========================================================================
%}
% extract parameters from param_vect
DPI_epsilon = param_vect{1};        % bound for damped pseudo inverse
beta_pos = param_vect{2};           % position buffer for un. constr.
beta_vel = param_vect{3};           % velocity buffer for un. constr.
lambda = param_vect{4};             % damping lambda for task N+1
K = param_vect{end};            % error gain vector


% algorithm step

% init vectors
Jra{N+1} = [];
T{N} = [];
e = cell(N,1);
h = cell(N,1);

% for p = N:-1:1     % e.g.: 3 2 1
%
%     % augmented jacobian
%     Jra{p} = [J{p}; Jra{p+1}];
%
%     % pseudoinverse
%     Jra_pinv{p} = damped_pseudo_inverse(Jra{p}, ...
%         DPI_lambda_max, ...
%         DPI_epsilon);
%
%     % rank-update
%     T{p} = rank_update_lincols(J{p}, Jra_pinv{p});
%
%     % error computation and xd computation
%     if unil_constr(p)
%         xd{p} = 0;
%     else
%         % compute orientation error, using quaternions
%         e{p,1} = [x_des_cur{p}{1} - x_cur{p}{1}; eo_using_quat(x_des_cur{p}{2}, x_cur{p}{2})];
%         % compute position er
%
%         % compute tasks vel
%
%         K_ = diag([K(p)*ones(6,1).']);
%         xd{p} = xd_des_cur{p} + K_*e{p};
%     end
%
% end
qdd{N+1} = -lambda*qd_prev;
% qdd{N+1} = lambda*qdd_prev;
qd{N+1} = qd_prev+Ts*qdd{N+1};
for p = N:-1:1     % e.g.: 3 2 1
    
    if unil_constr(p)   	% p is a unilateral constraint
        
        xd{p} = 0;
        
        % xd given by lower priority tasks, required
        % by unilateral_constr_activation_check
%         if(unil_constr(p)==1 || unil_constr(p) ==2)
%             xd_lower{p} = J{p}*(qd{p+1});
%             bp = beta_pos(1);           % position buffer for un. constr.
%             bv = beta_vel(1);           % velocity buffer for un. constr.
%             x_cons = x_cons_cur{p};
%         elseif(unil_constr(p)==3 || unil_constr(p)==4)
            xd_lower{p} = J{p}*qdd{p+1}+dJ{p}*qd{p+1};
            bp = beta_pos(2);           % position buffer for un. constr.
            bv = beta_vel(2);           % velocity buffer for un. constr.
%             for i=1:length(xd_lower{p})
%                             if(unil_constr(p) == 3)
%                                 x_cons(i) = min(x_cons_cur{p}(i),qd_prev(i)+1*Ts);
%                             else
%                                 x_cons(i) = max(x_cons_cur{p}(i),qd_prev(i)-1*Ts);
%             
%                             end
%                         end
            x_cons = x_cons_cur{p};
            
%         end
        for i=1:length(xd_lower{p})
            hj(i) = unilateral_constr_activation_check(  unil_constr(p), ...
                x_cur{p}(i), ...
                x_cons(i), ...
                xd_lower{p}(i), ...
                bp, ...
                bv);
            %             if(p == 1 || p==2)
%                                         hj(i) = 0;
            %             end
        end
        %         h{p} = max(hj);
        HJ = diag((hj>0));
        J{p} = HJ*J{p};
        J{p}( ~any(J{p},2), : ) = [];
        dJ{p} = HJ*dJ{p};
        dJ{p}( ~any(dJ{p},2), : ) = [];
        if(isempty(dJ{p}))
            dJ{p} = 0*J{p};
        end
        if(isempty(J{p}))
            Jra{p} = Jra{p+1};
            J_tilde = clean_jac(J{p}.', Jra{p+1}.').';
            %      disp('J_tilde = '); disp(J_tilde);
            
            if ~isempty(J_tilde)
                %pinv_J_tilde = damped_pseudo_inverse(J_tilde, DPI_lambda_max, DPI_epsilon);
                pinv_J_tilde = pinv(J_tilde, DPI_epsilon);
                Pt{p+1} = eye(size(J{p},2)) - pinv_J_tilde*J_tilde;
            else
                Pt{p+1} = eye(size(J{p},2));
            end
            
            qd{p} = qd{p+1};
            qdd{p} = qdd{p+1};
            continue;
        end
        Jra{p} = [J{p}; Jra{p+1}];
        
        J_tilde = clean_jac(J{p}.', Jra{p+1}.').';
        %      disp('J_tilde = '); disp(J_tilde);
        
        if ~isempty(J_tilde)
            %pinv_J_tilde = damped_pseudo_inverse(J_tilde, DPI_lambda_max, DPI_epsilon);
            pinv_J_tilde = pinv(J_tilde, DPI_epsilon);
            Pt{p+1} = eye(size(J{p},2)) - pinv_J_tilde*J_tilde;
        else
            Pt{p+1} = eye(size(J{p},2));
        end
        
        if(unil_constr(p)==1 || unil_constr(p) ==2)
            xdd{p} = -2*J{p}*qd_prev/Ts;
        elseif(unil_constr(p)==3 || unil_constr(p)==4)
            xdd{p} = 0;
        end
        h{p} = diag(hj);
        %          xdd{p}(~any(xdd{p},2)) = [];
    else
        
        
        % p is a task
        e{p,1} = [x_des_cur{p}{1} - x_cur{p}{1}; eo_using_quat(x_des_cur{p}{2}, x_cur{p}{2})];
        ev{p,1} = xd_des_cur{p}-J{p}*qd{p+1};
        % compute position er
        
        % compute tasks vel
        K = K{p};
        kp = K(1);
        kv = K(2);
        Kp_ = diag([kp*ones(6,1).']);
        Kv_ = diag([kv*ones(6,1).']);
        xdd{p} = xdd_des_cur{p} + 1*Kp_*e{p}+1*Kv_*ev{p};
        
        h{p} = 1;
        Jra{p} = [J{p}; Jra{p+1}];
        
        % pseudoinverse
        J_tilde = clean_jac(J{p}.', Jra{p+1}.').';
        %      disp('J_tilde = '); disp(J_tilde);
        
        if ~isempty(J_tilde)
            %pinv_J_tilde = damped_pseudo_inverse(J_tilde, DPI_lambda_max, DPI_epsilon);
            pinv_J_tilde = pinv(J_tilde, DPI_epsilon);
            Pt{p+1} = eye(size(J{p},2)) - pinv_J_tilde*J_tilde;
        else
            Pt{p+1} = eye(size(J{p},2));
        end
        
        
    end
    pinv_Jp_Pt = pinv(J{p}*Pt{p+1}, DPI_epsilon);
    qdd{p} = qdd{p+1} + h{p} * pinv_Jp_Pt * (xdd{p} - dJ{p}*qd{p+1}-J{p}*qdd{p+1});
    qd{p} = qd_prev+Ts*qdd{p};
end

% the real vel. is qd{1}
qd_des = qd{1};
qdd_des = qdd{1};

end
