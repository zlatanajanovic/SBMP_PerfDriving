%% plot surface for article
clear all;
close all;

load('surface_data_interp.mat');
load('surface_data.mat')

% s = surf(Beta_grid,Psidot_grid,v_eval_grid');
% alpha .3
% hold on; 

% D is your data
% Rescale data 1-64
C_radius_vector = min(C_radius_vector,1000);
d = log10(C_radius_vector);
mn = min(d(:));
rng = max(d(:))-mn;
d = 1+63*(d-mn)/rng; % Self scale data

sc = scatter3(x_data_4fit,y_data_4fit,z_data_4fit,[],d,'filled');

%image(d);

hC = colorbar;
L = [10 15 20 30 40 60 80 100 150 200 250 300 350 500 1000];
% Choose appropriate
% or somehow auto generate colorbar labels
l = 1+63*(log10(L)-mn)/rng; % Tick mark positions
set(hC,'Ytick',l,'YTicklabel',L);

%sc = scatter3(x_data_4fit,y_data_4fit,z_data_4fit,[],d,'filled');

colormap('jet');
xlabel('$\beta$','Interpreter','latex','FontSize',15);
ylabel('$\dot\psi$','Interpreter','latex','FontSize',15);
zlabel('$v$','Interpreter','latex','FontSize',15);
grid on;
%c = colorbar;


hold on; 
s = surf(Beta_grid,Psidot_grid,v_eval_grid', 'FaceColor','m', 'FaceAlpha', .3);
%alpha .3
xlim([-0.6 0.1]);
ylim([0 .75]);
zlim([0 45]);

hC.Location = 'manual';
hC.Label.String = 'Curvature [m]';
hC.AxisLocation = 'out';
hC.Label.FontSize = 13;
set(hC.Label,'Interpreter','latex')