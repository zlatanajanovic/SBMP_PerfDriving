function [state_glob, PAR] = loc2glob(state_loc, PAR)

state_glob=state_loc;

% Planar transformation : rotation and translation
phi=PAR.OPT.origin(3);


mRot = [cos(phi) -sin(phi);sin(phi) cos(phi)];


state_glob(:,1:2) = (mRot*state_loc(:,1:2)')';
state_glob(:,1:2) = state_glob(:,1:2) + repmat([PAR.OPT.origin(1), PAR.OPT.origin(2)], size(state_glob,1),1);
state_glob(:,3) = state_loc(:,3)+phi;