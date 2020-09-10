function [x,xd,xdd] = dualArmgenerateTrajectories(robot,q0,ee_right,ee_left,nSamplesRight,nSamplesLeft,Pright,qright,Pleft,qleft)
%DUALARMGENERATETRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

[x.right,xd.right,xdd.right] = generateTrajectories(nSamplesRight,q0,robot,ee_right,Pright,qright);
% T.right = repmat([P;quat.'],[1,400]);

[x.left,xd.left,xdd.left] = generateTrajectories(nSamplesLeft,q0,robot,ee_left,Pleft,qleft);

end

