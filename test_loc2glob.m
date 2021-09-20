

PAR.OPT.root.psi= pi/4;
PAR.OPT.root.x = 5;
PAR.OPT.root.y =10;

state_loc = [10, 0 , pi/2];

[state_glob, PAR] = loc2glob(state_loc, PAR);
state_glob