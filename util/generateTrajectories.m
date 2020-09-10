function [x,xd,xdd,p,q] = generateTrajectories(nsamples,joint0,robot,ee,pos_wp,quat_wp)
%GENERATETRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

T0 = getTransform(robot,joint0,ee);
P0 = T0(1:3,4);
R0 = T0(1:3,1:3);
q0 = rotm2quat(R0);
tfin = 1;
Ts = 1/(nsamples-1);
wayPoints = [P0 pos_wp];
timePoints = linspace(0,tfin, size(wayPoints,2));
tSamples = linspace(0,tfin, tfin/Ts+1);
[p,v,a,~] = quinticpolytraj(wayPoints,timePoints,tSamples);

wpOr = [q0;quat_wp];

t_rot = zeros(1,size(wpOr,1));
for i=2:size(wpOr,1)
    t_rot(i) = t_rot(i-1)+tfin/(size(wpOr,1)-1);
end

[s, sd, sdd] = quinticpolytraj([0 1],t_rot(1:2),linspace(t_rot(1),t_rot(2), diff(t_rot(1:2)/Ts)+1));
[q,w,alpha] = rottraj(wpOr(1,:),wpOr(2,:),t_rot(1:2),linspace(t_rot(1),t_rot(2), diff(t_rot(1:2)/Ts)+1),'TimeScaling',[s; sd; sdd]);
for i=2:size(wpOr,1)-1
    if(dot(wpOr(i,:),q(:,end)')<0)
        wpOr(i,:) = -wpOr(i,:);
    end
   [s, sd, sdd] = quinticpolytraj([0 1],t_rot(i:i+1),linspace(t_rot(i),t_rot(i+1), diff(t_rot(i:i+1)/Ts)+1));

    [q_aux, w_aux, alpha_aux] = rottraj(wpOr(i,:),wpOr(i+1,:),t_rot(i:i+1),linspace(t_rot(i),t_rot(i+1), diff(t_rot(i:i+1)/Ts)+1),'TimeScaling',[s; sd; sdd]);
    q = [q q_aux(:,2:end)];
    w = [w w_aux(:,2:end)];
    alpha = [w alpha_aux(:,2:end)];

end

x = [p;q];
xd = [v;w];
xdd = [a;alpha];
% for i=1:size(traj,2)-1
%     T{i} = trvec2tform(p(:,i)')*quat2tform(q(:,i)'); 
% end



