function [mot_prim_array,mot_prim_count, PAR] = mot_prim_bicycle(mot_prim_array, STATE, INPUTS, PAR)    
 %   function [mot_prim_array , mot_prim_count] = mot_prim_bicycle(mot_prim_array, STATE_XY, PAR.MOTPR.inputs , PAR);
 
%Function to return an array with motion primitives
% state FORMAT
% x, y, psi, beta, v, psidot


T_end = PAR.OPT.exp_dt;
Ts = 0.02;

    %% Initialize
    mot_prim_array_temp = zeros(PAR.OPT.exp_Nk, 6)+1e5;
    mot_prim_array = zeros(PAR.OPT.exp_Nk, 6);
    %% Motion primitives
    %   simple discretisation ZOH, TODO improve
    %   State: x, y, psi, beta, v, psidot
    %   Inputs: delta, acc
    
    %%sample model  for initial development - extremely wrong!!!
    
    
    delta_eval  = linspace( -PAR.VEH.delta_str_MAX , PAR.VEH.delta_str_MAX , PAR.OPT.exp_Nstr_bicycle);
    lambda_eval = linspace( -PAR.VEH.lambda_MAX , PAR.VEH.lambda_MAX  , PAR.OPT.exp_Na_bicycle);
    
    x0 =  STATE(1);
    y0 =  STATE(2);
    psi0 =  STATE(3);
    
    beta0 =  STATE(4);
    v0 = STATE(5);
    psidot0 = STATE(6);
    
   
    v_eval = v0;

    k=1;
    for i = 1:PAR.OPT.exp_Nstr_bicycle %Beta
        for j = 1:PAR.OPT.exp_Na_bicycle %psidot
    
%             %psi
%             mot_prim_array(:,3)= STATE(3)+ STATE(5)/PAR.VEH.len*PAR.OPT.exp_dt*INPUTS(:,2);
%             psi=(mot_prim_array(:,3)+STATE(3))/2;
%             %x  = xo + v cos t
%             mot_prim_array(:,1)= STATE(1) + STATE(5)*cos(psi)*PAR.OPT.exp_dt;
%             %y
%             mot_prim_array(:,2)= STATE(2)+ STATE(5)*sin(psi)*PAR.OPT.exp_dt;
% 
%             %beta=0
%             mot_prim_array(:,4)= STATE(4);
%             %v = v0+ acc t
%             mot_prim_array(:,5)= STATE(5)+ PAR.OPT.exp_dt* INPUTS(:,1);
%             %psidot
%             mot_prim_array(:,6);   
        
 

            invalidity_conditions =  isnan(v_eval) || v_eval > 42 ||  v_eval < 7 || (  abs(v_eval-v0) > 1 ) ;
            if invalidity_conditions 
                y_ij = 1e6*ones(1,6);
            else

                state0_bicycle = [x0,y0,psi0,v_eval, beta0, psidot0,  delta_eval(i), lambda_eval(j)];
                [t_ij,y_ij] = analytic_sol_bicycle(T_end,state0_bicycle, Ts, PAR);

            end

            mot_prim_array_temp(k,:)= [y_ij(end,1),y_ij(end,2),y_ij(end,3),y_ij(end,5),y_ij(end,4),y_ij(end,6)];
            k=k+1;
            
        end
    end
    
    idx_valid = abs(mot_prim_array_temp(:,3)) <1e3;
    mot_prim_count =sum(idx_valid);
    mot_prim_array(1:mot_prim_count,:) = mot_prim_array_temp(idx_valid,:);
    
end
 