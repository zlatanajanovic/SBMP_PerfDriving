function d_state = veh_model(t , state_0 )

params();

% veh parameters
% lf = veh_params (1);
% lr = veh_params (2);
% m = veh_params (3);
% Jz = veh_params (4);
% tau_delta = veh_params (5);
% tau_lambda_r = veh_params (6);

rho = 1.29;
Cx = 0.3;
Cy = 0.5;
c_roll = 0.01;

% Pacejka params
B_Pacjk = Pacejka_params(1);
C_Pacjk = Pacejka_params(2);
D_Pacjk = Pacejka_params(3);
E_Pacjk = Pacejka_params(4);

%initial condition and inputs
psi = state_0(3);
v = state_0(4);
beta = state_0(5);
psidot = state_0(6);
delta = state_0(7);
lambda_r = state_0(8);

delta_asymp = state_0(9);
lambda_r_asymp = state_0(10);

%%

% wheel lateral slips
alpha_f=(delta-atan((psidot*lf+v*sin(beta))/(v*cos(beta))));
alpha_r=(-atan((-psidot*lr+v*sin(beta))/(v*cos(beta))));
%  alpha_r=-atan((-psidot*lr+v*sin(beta))/(v*cos(beta)));
%  alpha_f=(-atan((+cos(delta)*psidot*lf+v*sin(beta-delta))/(v*cos(beta-delta)+psidot*lf*sin(delta))));


% vehicle acceleration
ax= -v*psidot*sin(beta); % v_delta/Ts*cos(beta)
ay=  psidot*v*cos(beta);

%% Longitudinal weight transfer using a steady-state weight transfer approach.
F_zf=9.81*m*lr/(lf+lr)-m*ax*h/(lf+lr);
F_zr=9.81*m*lf/(lf+lr)+m*ax*h/(lf+lr);

% longitudinal losses (aerodynamic, rollling resistance, etc.)

vx = v*cos(beta);
vy = v*sin(beta);
F_loss = 1/2*rho*Cx*vx^2 + (1-exp(-v^2))*c_roll*m*9.81+ 1/2*rho*Cy*vy^2;

% equivalent slips
sigma_sample = lambda_r/(1+lambda_r);
alpha_sample_f = tan(alpha_f)/(1+lambda_r);
alpha_sample_r = tan(alpha_r)/(1+lambda_r);
sigma_tot_f = max(0.0001,abs(alpha_sample_f));
sigma_tot_r =max(0.0001,sqrt( sigma_sample^2+alpha_sample_r^2 ));


%friction coefficients calculation
mux_f = 0; 
muy_f = alpha_sample_f/sigma_tot_f*D_Pacjk*sin(C_Pacjk*atan(B_Pacjk*(1-E_Pacjk)*(sigma_tot_f)+E_Pacjk*atan(B_Pacjk*(sigma_tot_f))));

mux_r = sigma_sample/sigma_tot_r*D_Pacjk*sin(C_Pacjk*atan(B_Pacjk*(1-E_Pacjk)*(sigma_tot_r)+E_Pacjk*atan(B_Pacjk*(sigma_tot_r))));
muy_r = alpha_sample_r/sigma_tot_r*D_Pacjk*sin(C_Pacjk*atan(B_Pacjk*(1-E_Pacjk)*(sigma_tot_r)+E_Pacjk*atan(B_Pacjk*(sigma_tot_r))));

%forces calculation
Fxf = F_zf * mux_f;
Fxr = F_zf * mux_r;
Fyf = F_zr * muy_f;
Fyr = F_zr * muy_r;


%%
dx = v * cos (psi + beta);
dy = v * sin (psi + beta);
d_psi = psidot;
dv = 1/m*( (Fxf)*cos(delta-beta) -(Fyf)*sin(delta-beta) + (Fxr)*cos(beta) +(Fyr)*sin(beta)-F_loss);
dbeta = 1/(m*v)*( (Fxf)*sin(delta-beta) +(Fyf)*cos(delta-beta) - (Fxr)*sin(beta) +(Fyr)*cos(beta))-psidot;
dd_psi = 1/Jz * (lf*((Fxf)*sin(delta)+(Fyf)*cos(delta)) -lr* (Fyr));
d_delta = 1/tau_delta * (delta_asymp-delta);
d_lambda_r = 1/tau_lambda_r * (lambda_r_asymp-lambda_r);

d_state = [dx; dy; d_psi; dv ; dbeta ; dd_psi; d_delta; d_lambda_r; 0 ; 0];