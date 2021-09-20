%test interpolation function
close all;
load('surface_data');

beta_vect = -0.6:0.02:0.1;
psidot_vect = 0:0.02:0.72;

v_eval_grid = zeros(length(beta_vect),length(psidot_vect));
delta_eval_grid = zeros(length(beta_vect),length(psidot_vect));
lambda_eval_grid = zeros(length(beta_vect),length(psidot_vect));
percentage=0

warning('off','all')
for i = 1:length(beta_vect)
    for j = 1:length(psidot_vect)

        v_eval_grid(i,j) =      griddata(x_data_4fit,y_data_4fit,z_data_4fit ,beta_vect(i), psidot_vect(j));
        delta_eval_grid(i,j) =  griddata(x_data_4fit,y_data_4fit,z1_data_4fit,beta_vect(i), psidot_vect(j));
        lambda_eval_grid(i,j) = griddata(x_data_4fit,y_data_4fit,z2_data_4fit,beta_vect(i), psidot_vect(j));

        
        percentage = ((i-1)*length(beta_vect)+j) /(length(beta_vect)*length(psidot_vect))
    end
end

warning('on','all')
[Beta_grid,Psidot_grid]=meshgrid(beta_vect,psidot_vect);
figure;
surf(Beta_grid,Psidot_grid,v_eval_grid');
figure;
surf(Beta_grid,Psidot_grid,delta_eval_grid');
figure;
surf(Beta_grid,Psidot_grid,lambda_eval_grid');

save('surface_data_interp','Beta_grid' ,'Psidot_grid' ,'v_eval_grid', 'delta_eval_grid', 'lambda_eval_grid');

%%

interp2(Beta_grid ,Psidot_grid ,v_eval_grid',-0.2,.3)
