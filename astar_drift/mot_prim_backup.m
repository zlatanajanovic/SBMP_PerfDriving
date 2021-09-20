function [mot_prim_array, PAR] = mot_prim(mot_prim_array, STATE, INPUTS, PAR)    

%Function to return an array with motion primitives
% state FORMAT
% x, y, psi, beta, v, psidot


    %% Initialize
    mot_prim_array = zeros(PAR.OPT.exp_Nk, 6);
    %% Motion primitives
    %   simple discretisation ZOH, TODO improve
    %   State: x, y, psi, beta, v, psidot
    %   Inputs: delta, acc
    
    %%sample model  for initial development - extremely wrong!!!
    
    %psi
    mot_prim_array(:,3)= STATE(3)+ STATE(5)/PAR.VEH.len*PAR.OPT.exp_dt*INPUTS(:,2);
    psi=(mot_prim_array(:,3)+STATE(3))/2;
    %x  = xo + v cos t
    mot_prim_array(:,1)= STATE(1) + STATE(5)*cos(psi)*PAR.OPT.exp_dt;
    %y
    mot_prim_array(:,2)= STATE(2)+ STATE(5)*sin(psi)*PAR.OPT.exp_dt;
    
    %beta=0
    mot_prim_array(:,4)= STATE(4);
    %v = v0+ acc t
    mot_prim_array(:,5)= STATE(5)+ PAR.OPT.exp_dt* INPUTS(:,1);
    %psidot
    mot_prim_array(:,6);   
        
end
 