function [t_ij,y_ij] = analytic_sol(T_end,state_0, Ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

state = state_0';

t_ij = zeros(round(T_end/Ts),1);
y_ij = zeros(round(T_end/Ts),length(state_0));
y_ij(1,:) = state_0;

for i = 1: T_end/Ts-1

    d_state = veh_model(0, state );
    state = state + d_state*Ts;
    t_ij(i+1) = i*Ts;
    y_ij(i+1,:) = state';

end

