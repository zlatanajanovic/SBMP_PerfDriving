function  d_state  = traj_model(  state, inputs , DeltaBeta)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

x_dot = inputs(1)*cos(inputs(2) + state(3));
y_dot = inputs(1)*sin(inputs(2) + state(3));

d_state = [x_dot, y_dot , (inputs(3)-.5*DeltaBeta)];


end

