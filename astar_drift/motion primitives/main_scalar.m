clear all;
clc;
close all;

x0 = 0;
y0 = 0;
psi0 = 0;

v0 = 20;
beta0 = 0;
psidot0 = 0;
delta0 = 0;
lambda_r0 = 0.001;
delta_asymp0 = .02;
lambda_r_asymp0 = .1;

%N_mp = 10;
c_map = [1 0 0];

state_0 = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_asymp0, lambda_r_asymp0];
[t,y] = ode45(@veh_model,[0 10],state_0);

%%
figure;
subplot(3,1,1);
plot(t,y(:,4));
ylabel('v');
subplot(3,1,2);
plot(t,y(:,5));
ylabel('beta');
subplot(3,1,3);
plot(t,y(:,6));
ylabel('psidot');

%%
figure;
subplot(2,1,1);
plot(t,y(:,7));
ylabel('delta');
subplot(2,1,2);
plot(t,y(:,8));
ylabel('lambda r');


%%
% sample_freq  = 10;


figure;

%find indices for uniform display of vehicle positions

differences = [y(:,1:2);y(end,1:2)]-[0,0;y(:,1:2)];
dist = sqrt(differences(:,1).^2+differences(:,2).^2);
dist(1)=[];

idx = [];
j = 1;
while j < length(dist)
    id = min(find(cumsum(dist(j:end))>5));
    idx = [idx; id+j];
    j = id+j+1
end

pt_x = y(idx,1);
pt_y = y(idx,2);
pt_psi  = y(idx,3);
pt_v    = y(idx,4);
pt_beta = y(idx,5);

angles = pt_psi+pt_beta;
%angles = pt_psi;
scale_fact = 4;

for ii = 1:length(pt_x)-1
draw_rectangle(pt_x(ii),pt_y(ii),pt_psi(ii),c_map);
quiver( pt_x(ii) , pt_y(ii) , pt_v(ii)*cos(angles(ii))/scale_fact , pt_v(ii)*sin(angles(ii))/scale_fact ,'k', 'LineWidth',1.5 , 'MaxHeadSize',  2 );
alpha(.2) ;
hold on;
end
axis equal
