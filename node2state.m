function [STATE] = node2state( node, PAR)

%Function to return an Continous state from node
%This function takes a node and returns continous state
%node format
% x, y, psi, beta, v, psidot
% costs (1-2)|state(3-8) | parent (9-14) | reman(15-20) | time
%--------------------------------------------------------------------------
%  1  |  2  | 3 | 4 |...| 8 | 9  |...|  14| 15 | ... | 20 |21
%--------------------------------------------------------------------------
% f(n)| g(n)|x1 |x2 |...|x6 |par1|...|par6|rem1| ... |rem6|
%           |ind|ind|...|ind|ind |...|ind |ind | ... |ind |ind
%--------------------------------------------------------------------------
% Note: - d rounded to previous for dt cutting trajectories
%       - All nodes explored - improvement possible using amax

% STATE = [];
% 
% STATE(1,:) =  PAR.OPT.dx*(node(:,3)-1) + node(:,15);
% STATE(2,:) =  PAR.OPT.dy*(node(:,4)-1-(PAR.OPT.Ny-1)/2) + node(:,16);
% STATE(3,:) =  PAR.OPT.dpsi*(node(:,5)-1-(PAR.OPT.Npsi-1)/2) + node(:,17);
% STATE(4,:) =  PAR.OPT.dbeta*(node(:,6)-1-(PAR.OPT.Nbeta-1)/2) + node(:,18);
% STATE(5,:) =  PAR.OPT.dv*(node(:,7)-1) + node(:,19);
% STATE(6,:) =  PAR.OPT.dpsidot*(node(:,8)-1-(PAR.OPT.Npsidot-1)/2) + node(:,20);

STATE= zeros(1,6);

STATE(1) =  PAR.OPT.dx*(node(:,3)-1) + node(:,15);
STATE(2) =  PAR.OPT.dy*(node(:,4)-1-(PAR.OPT.Ny-1)/2) + node(:,16);
STATE(3) =  PAR.OPT.dpsi*(node(:,5)-1-(PAR.OPT.Npsi-1)/2) + node(:,17);
STATE(4) =  PAR.OPT.dbeta*(node(:,6)-1-(PAR.OPT.Nbeta-1)/2) + node(:,18);
STATE(5) =  PAR.OPT.dv*(node(:,7)-1) + node(:,19);
STATE(6) =  PAR.OPT.dpsidot*(node(:,8)-1-(PAR.OPT.Npsidot-1)/2) + node(:,20);