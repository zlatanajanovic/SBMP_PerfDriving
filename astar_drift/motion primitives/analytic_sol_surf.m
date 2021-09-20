function [t_ij,y_ij] = analytic_sol_surf(T_end,state0_surf, Ts)

% state0_surf = [x0,y0,psi0,v0, beta0, psidot0, v_eval, beta_eval(i), psidot_eval(i)];

%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

inputs0 = state0_surf(4:6); %[v0, beta0, psidot0]';
inputs_end = state0_surf(7:9);

N_iterations = T_end/Ts-1;

state = state0_surf(1:3)';

t_ij = zeros(round(T_end/Ts),1);
y_ij = zeros(round(T_end/Ts),6);
y_ij(1,:) = [state; inputs0'];

N_iter_input = 10;
beta_old = inputs0(2);

for i = 1: N_iterations 
    
    
    inputs = inputs0 + (inputs_end-inputs0)*min(1,(i/N_iter_input));
    
    d_state = traj_model( state, inputs', (inputs(2)-beta_old)/Ts );
    state = state + Ts*d_state';
    t_ij(i+1) = i*Ts;
    y_ij(i+1,:) = [state; inputs'];
    
    beta_old = inputs(2);
end

