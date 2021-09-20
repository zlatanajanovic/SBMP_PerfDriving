function[plt_ego]= draw_rectangle(x_c,y_c,theta, c_map, PAR,alpha)
% Serve per plottare i rettangoli inclinati che rappresentano l'auto
coder.extrinsic('fill')
L = PAR.SIM.vehicle_lngth;
H = PAR.SIM.vehicle_width;

X = [-L/2 L/2 L/2 -L/2 ]; 
Y = [-H/2 -H/2 H/2 H/2 ]; 
cth = cos(theta) ; 
sth = sin(theta); 
Xrot =  X*cth - Y*sth;
Yrot =  X*sth + Y*cth;

%fill(Xrot + x_c, Yrot+y_c,'r')
plt_ego=fill(Xrot + x_c, Yrot+y_c,c_map,'facealpha',alpha, 'edgecolor','k');

end