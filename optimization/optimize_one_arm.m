function [qr,t_resamp]=optimize_one_arm(q_R,s,time_step,robot,scale)
import casadi.*

qd_bound = robot.limits.velocity.max;
qdd_bound_max = robot.limits.acceleration.max;
qddd_bound_max = robot.limits.jerk.max;

qdd_bound_min = robot.limits.acceleration.min;
qddd_bound_min = robot.limits.jerk.min;

if nargin==5
    if scale < 1
        scale = 1;
    end
    qd_bound = qd_bound./scale;
    qdd_bound_max = qdd_bound_max./scale;
    qdd_bound_min = qdd_bound_min./scale;
    
end
% s = 0:1/(size(q_R,2)-1):1;
sm = (s(2:end)+s(1:end-1))/2;


ds = s(2:end)-s(1:end-1);
% a = sdpvar((size(sm,2)),1,'full');
% b = sdpvar((size(s,2)),1,'full');
% x = [b;a];
q_full = [q_R];
% for i=1:size(q_full,1)
% %     q(i,:) = interp1(s,q_full(i,:),sm,'pchip');
%
% qp(i,:) = gradient(q(i,:),sm);
% qpp(i,:) = gradient(qp(i,:),sm);
% end
% [ qm, qp, qpp, ~] = bsplinepolytraj( q_full,s,saug);
for i=1:size(q_full,1)
    fff{i} = fit(s.',q_full(i,:).','smoothingspline','SmoothingParam',1-1e-16);
    qm(i,:) = fff{i}(sm);
    qp(i,:) = gradient(qm(i,:),sm);
    qpp(i,:) = gradient(qp(i,:),sm);
  
end
b_bound = [];
% [ ~, qps, ~, ~] = quinticpolytraj( q_full,s,s);
for i=1:size(q_full,1)
    qps(i,:) = gradient(q_full(i,:),s);
end
% [ qs, qps, qpps, pp] = quinticpolytraj( q_full,s,s);

for i=1:size(qps,2)
    b_bound(i) = min((qd_bound.^2).'./qps(:,i).^2);
end
b_bound(1) = 1e-15;
b_bound(end) = 1e-15;
opti = casadi.Opti();



b = opti.variable((size(s,2)));

% acc = opti.variable(14, size(sm,2));
% dacc = opti.variable(14, size(sm,2)-1);


acc = qp*diag((b(2:end)-b(1:end-1))./(2*ds.'))+qpp*diag((b(1:end-1)+b(2:end))/2);
opti.subject_to(qdd_bound_min.' <= acc <= qdd_bound_max.');


% ff = 0;
dt = 2*ds./((b_bound(1:end-1)).^(0.5)+(b_bound(2:end)).^0.5);
% dt = 2*ds./((b(1:end-1)).^(0.5)+(b(2:end)).^0.5).';

ff = sum(2*ds./((b(1:end-1)).^(0.5)+(b(2:end)).^0.5).');
opti.minimize( ff);

opti.subject_to(qddd_bound_min.' <= 2*acc(:,1)/dt(1) <= qddd_bound_max.');
for i =1:size(q_R,1)
    opti.subject_to(qddd_bound_min(i) <=(acc(i,2:end)-acc(i,1:end-1))./(dt(1:end-1)/2+dt(2:end)/2)<=qddd_bound_max(i));
end
opti.subject_to(qddd_bound_min.' <= acc(:,end)/(dt(end)/2) <= qddd_bound_max.');

%     opti.subject_to(-qddd_bound' <= acc(:,end) <= qddd_bound');
% for i =1:7
%     opti.subject_to(-qddd_bound<=(acc(i,2:end)-acc(i,1:end-1))<=qddd_bound);
% end
%     opti.subject_to(-qddd_bound' <= acc(:,end) <= qddd_bound');
%

opti.subject_to(     b_bound'>=b>=1e-17 );
opti.set_initial(b, b_bound);
opti.subject_to(       b(1) == 1e-15 );
opti.subject_to(       b(end) == 1e-15 );
% opti.solver('ipopt');
p_opts = struct('expand',true);
s_opts = struct('linear_solver', 'ma57','jac_d_constant','yes','jac_c_constant','yes','fast_step_computation','yes');
opti.solver('ipopt',p_opts,s_opts);%,p_opts,s_opts);


sol = opti.solve();

% sol = optimize(Constraints,Objective,ops);

%%
bk = sol.value(b);
ak = (bk(2:end)-bk(1:end-1))./(2*ds.');
t = 0;
t = cumsum(2*ds.'./(sqrt(bk(2:end))+sqrt(bk(1:end-1))));

t_tot = [0 t.'];
t_resamp = 0:time_step:t_tot(end);

for i=1:size(t_resamp,2)
    s_res_pos(i) = s_resamp(s,t_tot,bk,ak,t_resamp(i), 1); %indice 1 con posizioni, 2 lineare, 3 fwd integration
    %     size(t_resamp,2)-i;
end

% for i = 1:7
% qr(i,:) = interp1(s,q_R(i,:),s_res_pos,'pchip');
% end
for i=1:size(q_full,1)
    qr(i,:) = fff{i}(s_res_pos);
end
% qr = quinticpolytraj(q_full,s,s_res_pos);
% if(isequal(qr(:,end),q_R(:,end)))
% else
%     qr = [qr q_R(:,end)];
%     t_resamp = [t_resamp t_resamp(end)+time_step];
% end
end