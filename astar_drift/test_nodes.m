vi = 10;

ds = 5;
dt = 0.5;
dv = 0.3;
vmax = 25;
amax = 2;


vf=0:dv:vmax;


% trajektories which achieve vf at ds
t_ds= 2*ds./(vi+vf);
a_ds= (vf-vi)./t_ds;

% trajectories which achieve vf at dt
a_dt= (vf-vi)./dt;
t_dt= 0*a_dt + dt;

% choosing of trajecotories
vx=2*ds/dt-vi;

a_ds(vf<vx)=0;
t_ds(vf<vx)=0;
a_dt(vf>=vx)=0;
t_dt(vf>=vx)=0;


ai=a_ds+a_dt;
ti=t_ds+t_dt;

% ai(ai>=amax)=-inf;
% ai(ai<=-amax)=-inf;
% ti(ai>=amax)=0;
% ti(ai<=-amax)=0;


k_plot = 50;
kti= 0:1: k_plot-1;
t_plot = kti'*ti/(k_plot-1);
t_plot = t_plot';
a_ki = repmat ( ai' , [1, k_plot]);


vi_mat= repmat(vi, [length(vf), k_plot]);
si= a_ki.*t_plot.^2./2 + vi_mat.*t_plot;

% vi_mat= repmat(vi, [length(vf),1]);
% si= transpose(ai)*t_plot.^2./2 + vi_mat*t_plot;

sk = ai.*ti.^2/2 + vi.*ti;

figure
plot ( t_plot', si' )
hold on
plot( [0 0 dt dt 0], [0 ds ds 0 0])

plot(ti, sk, 'o');

xlabel('time')
ylabel('distance')
ylim([-0.1*ds, ds*1.1])
xlim([-0.1*dt, dt*1.1])