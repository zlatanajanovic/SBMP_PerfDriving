%% plot surface for article
clear all;
close all;

load('surface_data_interp.mat');
load('surface_data.mat')

% % s = surf(Beta_grid,Psidot_grid,v_eval_grid');
% % alpha .3
% % hold on; 
% 
% % D is your data
% % Rescale data 1-64
% C_radius_vector = min(C_radius_vector,1000);
% d = log10(C_radius_vector);
% mn = min(d(:));
% rng = max(d(:))-mn;
% d = 1+63*(d-mn)/rng; % Self scale data
% 
% sc = scatter3(x_data_4fit,y_data_4fit,z_data_4fit,[],d,'filled');
% 
% %image(d);
% 
% hC = colorbar;
% L = [10 15 20 30 40 60 80 100 150 200 250 300 350 500 1000];
% % Choose appropriate
% % or somehow auto generate colorbar labels
% l = 1+63*(log10(L)-mn)/rng; % Tick mark positions
% set(hC,'Ytick',l,'YTicklabel',L);
% 
% %sc = scatter3(x_data_4fit,y_data_4fit,z_data_4fit,[],d,'filled');
% 
% colormap('jet');
% grid on;
% %c = colorbar;
% 
% 
% hold on; 
s = surf(Beta_grid,Psidot_grid,v_eval_grid', 'FaceColor','m', 'FaceAlpha', .3);
%alpha .3
xlim([-0.6 0.1]);
ylim([0 .75]);
zlim([0 45]);

xlabel('$\beta$','Interpreter','latex','FontSize',15);
ylabel('$\dot\psi$','Interpreter','latex','FontSize',15);
zlabel('$v$','Interpreter','latex','FontSize',15);
hold on;

%%
x0 = [-0.1; 0.32; 16.1+0.5];
beta0 = x0(1);
psidot0 = x0(2);
betaD    = .1;
psidotD  = .15;
v0 = x0(3);


n_psidot=11;
psidot_kern = linspace( psidot0-psidotD/(2^((n_psidot-5)/2)) , psidot0+psidotD/(2^((n_psidot-5)/2)) , 5);
if n_psidot-5==0
    psidot_eval =psidot_kern;
else
    adj = zeros(1,(n_psidot-5)/2);
    for i =1: length(adj)
        adj(i) = psidotD/(2^(i-1));
    end
    psidot_eval = [psidot0-adj , psidot_kern , psidot0+fliplr(adj)];
end
%
%
n_beta = 9;
beta_kern = linspace( beta0-betaD/(2^((n_beta-5)/2)) , beta0+betaD/(2^((n_beta-5)/2)) , 5);
if n_beta-5==0
    beta_eval =beta_kern;
else
    adj = zeros(1,(n_beta-5)/2);
    for i =1: length(adj)
        adj(i) = betaD/(2^(i-1));
    end
    beta_eval = [beta0-adj , beta_kern , beta0+fliplr(adj)];
end

[beta_grid, psidot_grid] = meshgrid(beta_eval,psidot_eval);

vgrid = zeros( size(beta_grid') );

for i =1:length(beta_eval)
    for j =1:length(psidot_eval)
   
        %vgrid(i,j) =interp2(Beta_grid ,Psidot_grid ,v_eval_grid',sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
        vgrid(i,j) =interp2(Beta_grid ,Psidot_grid ,v_eval_grid',beta_eval(i), psidot_eval(j));
    end
end

vgrid_t = vgrid';
beta_vect = beta_grid(:);
psidot_vect = psidot_grid(:);
v_vect = vgrid_t(:);

idx_valid = abs(v_vect-v0)<2;
%*ones(length(v_vect),1)) < 2;


scatter3(beta_vect,psidot_vect, v_vect ,'MarkerFaceColor','b');
hold on;
scatter3(beta_vect(idx_valid),psidot_vect(idx_valid), v_vect(idx_valid), 'filled','MarkerFaceColor','r');
hold on;
scatter3(beta0,psidot0,v0,'filled' ,'MarkerFaceColor','k');
legend('Equilibrium States Manifold','valid expanded nodes','invalid expanded nodes','current node','Location','East');

xlim([-0.3 0.1]);
box on;
% 
% hC.Location = 'manual';
% hC.Label.String = 'Curvature [m]';
% hC.AxisLocation = 'out';
% hC.Label.FontSize = 13;
% set(hC.Label,'Interpreter','latex')