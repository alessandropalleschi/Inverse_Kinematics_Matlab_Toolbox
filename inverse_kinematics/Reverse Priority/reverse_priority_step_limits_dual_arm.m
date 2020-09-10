function [qd_des,h_lim] = reverse_priority_step_limits_dual_arm(N, qd_prev, x_des_cur, xd_des_cur, unil_constr, x_cons_cur, param_vect, J, x_cur)
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
DPI_lambda_max = param_vect(1);     % damping for damped pseudo inverse
DPI_epsilon = param_vect(2);        % bound for damped pseudo inverse
beta_pos = param_vect(3);           % position buffer for un. constr.
beta_vel = param_vect(4);           % velocity buffer for un. constr.
lambda = param_vect(5);             % damping lambda for task N+1
K = param_vect(6:end);            % error gain vector


% algorithm step
% Jra = cell(N+1,1);
% T = cell(N,1);
% qd = cell(N+1,1);
% xd = cell(N,1);
% xd_lower = cell(N,1);
% Jra_pinv = cell(N,1);
% h_lim = cell(N,1);
% x_cur = {{zeros(14,1)};{zeros(14,1)};{cell(4,1)}};
% x_cur = x_curi;
% x_cons_cur = cell(N,1);
% x_cons_cur = x_cons_curi;

% J = {{zeros(14,14)} {zeros(14,14)} {zeros(12,14)}};
% J = Ja;
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
qd{N+1} = lambda *qd_prev;

for p = N:-1:1     % e.g.: 3 2 1
    
    if unil_constr(p)   	% p is a unilateral constraint
        
        xd{p} = 0;
        % xd given by lower priority tasks, required
        % by unilateral_constr_activation_check
        %         test = J{p}*(qd{p+1});
        xd_lower{p} = J{p}*(qd{p+1});
        hj = zeros(length(xd_lower{p}),1);
        %         cur = cell(1);
        %         cur = x_cur{p};
        for i=1:length(xd_lower{p})
            hj(i) = unilateral_constr_activation_check(  unil_constr(p), ...
                x_cur{p}(i), ...
                x_cons_cur{p}(i), ...
                xd_lower{p}(i), ...
                beta_pos, ...
                beta_vel);
        end
        %         h_lim{p} = hj;
        h{p} = 0*diag(hj);
        HJ = diag((hj>0));
        J{p} = HJ*J{p};
        J{p}( ~any(J{p},2), : ) = [];
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
        
        
    else
        
        
        % p is a task
        e{p,1} = [x_des_cur{p}{1} - x_cur{p}{1}; eo_using_quat(x_des_cur{p}{2}, x_cur{p}{2});x_des_cur{p}{3} - x_cur{p}{3}; eo_using_quat(x_des_cur{p}{4}, x_cur{p}{4})];
        % compute position er
        
        % compute tasks vel
        
        K_ = diag([K(p)*ones(12,1).']);
        xd{p} = xd_des_cur{p} + K_*e{p};
        
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
    %     psInv = pinv(J{p}*T{p});
    qd{p} = qd{p+1} + h{p} * pinv_Jp_Pt * (xd{p} - J{p}*qd{p+1});
end

% the real vel. is qd{1}
qd_des = qd{1};

end
