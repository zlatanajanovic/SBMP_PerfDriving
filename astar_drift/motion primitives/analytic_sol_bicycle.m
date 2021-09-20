function [t_ij,y_ij] = analytic_sol_bicycle(T_end,state0_bicycle, Ts, PAR)

% state0_surf    = [x0,y0,psi0,v0, beta0, psidot0, v_eval, beta_eval(i), psidot_eval(i)];
% state0_bicycle = [x0,y0,psi0,v0, beta0, psidot0,  delta_eval(i), lambda_eval(j)];


%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

dyn_states0 = state0_bicycle(4:6); %[v0, beta0, psidot0]';
inputs = state0_bicycle(7:8);

N_iterations = T_end/Ts-1;

state = [state0_bicycle(1:3)'; dyn_states0'];

t_ij = zeros(round(T_end/Ts),1);
y_ij = zeros(round(T_end/Ts),6);
y_ij(1,:) = state;


for i = 1: N_iterations 
   
    d_state = traj_model_bicycle( state, inputs',PAR );
    state = state + Ts*d_state';
    t_ij(i+1) = i*Ts;
    y_ij(i+1,:) = state;
    
end

