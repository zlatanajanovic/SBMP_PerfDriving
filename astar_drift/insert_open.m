function [new_row , PAR] = insert_open(fn, gn, STATE_REP, PAR, DAT)
%Function to Populate the OPEN LIST
%NODE FORMAT
%--------------------------------------------------------------------------
%  1  | 2 | 3 | 4 | 5 |  6  |  7  |  8  |  9   | 10  |  11 |  12 | 13 |  14
%--------------------------------------------------------------------------
% f(n)| d | v | t | l | par | par | par | par  |g(n) |del_d|del_t| t_l| t_r
% [kJ]|ind|ind|ind|ind| d-in| v-in| t-in| l-in |[kJ] | (m) | (s) | (s)| (s)
%--------------------------------------------------------------------------




    new_row=zeros(1,21);
    parnode=zeros(1,21);
    parnode(1:8)= [0, 0, -1, -1, -1, -1, -1, -1];
    STATE_REP_fr = xy2frenet(STATE_REP, 1, PAR, PAR.OPT.root_wp);
    [new_row , PAR] = state2node(parnode, STATE_REP_fr, 1, PAR);%, DAT);
    
    new_row(1,1) = fn;% f(n) %TODO improve initial
    new_row(1,2) = gn;% g(n)
    new_row(1,21) = 0;% time cout

end