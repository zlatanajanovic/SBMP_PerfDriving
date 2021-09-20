function  d_state  = traj_model_bicycle(  state, inputs ,PAR)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%state = [x0,y0,psi0,v0, beta0, psidot0]

x_dot = state(4)*cos(state(5) + state(3));
y_dot = state(4)*sin(state(5) + state(3));
psi_dot = state(6);

Fyf = -PAR.VEH.Cyf*(state(5) + state(6)*PAR.VEH.lf/(max(2,state(4)))  -inputs(1) );
Fyr= -PAR.VEH.Cyr*(state(5) - state(6)*PAR.VEH.lr/(max(2,state(4)))   );

v_dot = 1/PAR.VEH.m*PAR.VEH.Cxr*inputs(2);
beta_dot = 1/(PAR.VEH.m*state(4))*(Fyf+Fyr)-state(6);
psi_ddot = 1/PAR.VEH.Jz *(PAR.VEH.lf*Fyf-PAR.VEH.lr*Fyr);

d_state = [x_dot, y_dot , psi_dot,v_dot ,beta_dot , psi_ddot ];


end

