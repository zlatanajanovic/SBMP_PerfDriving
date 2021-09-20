clear all;
clc;
close all;

T_end = 5;

x0 = 0;
y0 = 0;
psi0 = 0;

% v0 = 20;
% beta0 = 0;
% psidot0 = 0;
% delta0 = 0;
% lambda_r0 = 0.005;
v0 = 21;
beta0 = -0.1;
psidot0 = 0.35;
%delta0 = 0.2;
%lambda_r0 = 0.1;


% delta_asymp0 = .5;
% lambda_r_asymp0 = .4;

p00 =       15.46;%  (14.87, 16.05)
p10 =      -699.4;%  (-707.9, -690.8)
p01 =       108.9;%  (90.98, 126.8)
p20 =       -2626;%  (-2658, -2594)
p11 =        3975;%  (3854, 4095)
p02 =       -1602;%  (-1790, -1414)
p30 =       -4957;%  (-5057, -4857)
p21 =        9505;%  (9322, 9688)
p12 =       -9802;%  (-1.045e+04, -9158)
p03 =        7222;%  (6351, 8092)
p40 =       -4528;%  (-4692, -4365)
p31 =   1.098e+04;%  (1.074e+04, 1.122e+04)
p22 =  -1.216e+04;%  (-1.265e+04, -1.167e+04)
p13 =   1.233e+04;%  (1.081e+04, 1.384e+04)
p04 =  -1.384e+04;%  (-1.566e+04, -1.201e+04)
p50 =       -1585;%  (-1691, -1478)
p41 =        4835;%  (4649, 5021)
p32 =       -5927;%  (-6236, -5619)
p23 =        5938;%  (5337, 6539)
p14 =       -6269;%  (-7612, -4926)
p05 =        9663;%  (8253, 1.107e+04)

params = [p00 p01 p02 p03 p04 p05 p10 p11 p12 p13 p14 p20 p21 p22 p23 p30 p31 p32 p40 p41 p50];

p00_delta =   -0.006313 ;% (-0.006896, -0.00573)
p10_delta =      0.6077 ;% (0.594, 0.6215)
p01_delta =      0.2875 ;% (0.2849, 0.2901)
p20_delta =       7.511 ;% (7.399, 7.622)
p11_delta =     -0.6218 ;% (-0.6727, -0.5709)
p30_delta =       30.79 ;% (30.41, 31.18)
p21_delta =      -9.389 ;% (-9.695, -9.082)
p40_delta =       44.14 ;% (43.54, 44.74)
p31_delta =      -22.62 ;% (-23.31, -21.93)
p50_delta =       22.01 ;% (21.65, 22.36)
p41_delta =      -16.36 ;% (-16.87, -15.85)

params_delta = [p00_delta p01_delta p10_delta p11_delta p20_delta p21_delta p30_delta p31_delta p40_delta p41_delta p50_delta];

p00_lambda =    0.003737 ;%  (0.002766, 0.004708)
p10_lambda =      0.1443 ;%  (0.14, 0.1487)
p01_lambda =     0.06823 ;%  (0.06499, 0.07148)
p20_lambda =       2.183 ;%  (2.177, 2.188)
p11_lambda =     -0.2336 ;%  (-0.2416, -0.2256)

params_lambda = [p00_lambda p01_lambda p10_lambda p11_lambda p20_lambda ];

N_mp = 8;
c_map = jet(N_mp);

betaD = .1;
psidotD = .1;

beta_eval = linspace(beta0-betaD,beta0+betaD,N_mp/2); 
psidot_eval = linspace(psidot0-psidotD,psidot0+psidotD,N_mp/2); 


%eval(['state_0_' i '_' j ' = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_eval(' i '), lambda_r_eval(' j ')];']);

t_tot = cell(N_mp/2);
y_tot = cell(N_mp/2);
sampled_data = cell(N_mp/2);

Ts = 0.02;

tic
for i = 1:N_mp/2
    for j = 1:N_mp/2
        
        %state_0 = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_eval(i), lambda_r_eval(j)];
        
        [ v_eval , delta_eval, lambda_eval] = v_of_beta_psidot(sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)), params, params_delta, params_lambda);
        state0_surf = [x0,y0,psi0, v0, beta0 , psidot0 , v_eval, beta_eval(i), psidot_eval(j)];
        
        
        %ode45 solution
        %[t_ij,y_ij] = ode45(@veh_model,[0,T_end],state_0);
        %analytic solution
        [t_ij,y_ij] = analytic_sol_surf(T_end,state0_surf,Ts);
        t_tot(i,j) = {t_ij};
        y_tot(i,j) = {[y_ij, delta_eval*ones(length(t_ij),1) , lambda_eval*ones(length(t_ij),1)]};
        
    end
end
toc
%%
figure;
subplot(3,1,1);
for i = 1:N_mp/2
    for j = 1:N_mp/2
        t_ij = t_tot{i,j};
        y_ij = y_tot{i,j};
        plot(t_ij,y_ij(:,4));
        hold on;
    end
end
ylabel('v');

subplot(3,1,2);
for i = 1:N_mp/2
    for j = 1:N_mp/2
        t_ij = t_tot{i,j};
        y_ij = y_tot{i,j};
        plot(t_ij,y_ij(:,5));
        hold on;
    end
end
ylabel('beta');

subplot(3,1,3);
for i = 1:N_mp/2
    for j = 1:N_mp/2
        t_ij = t_tot{i,j};
        y_ij = y_tot{i,j};
        plot(t_ij,y_ij(:,6));
        hold on;
    end
end
ylabel('psidot');

%
figure;
subplot(2,1,1);
for i = 1:N_mp/2
    for j = 1:N_mp/2
        t_ij = t_tot{i,j};
        y_ij = y_tot{i,j};
        plot(t_ij,y_ij(:,7));
        hold on;
    end
end
ylabel('delta');
subplot(2,1,2);
for i = 1:N_mp/2
    for j = 1:N_mp/2
        t_ij = t_tot{i,j};
        y_ij = y_tot{i,j};
        plot(t_ij,y_ij(:,8));
        hold on;
    end
end
ylabel('lambda r');


%%
%sample_freq  = 20;

for i = 1:N_mp/2
    for j = 1:N_mp/2
        y = y_tot{i,j};
        
        differences = [y(:,1:2);y(end,1:2)]-[0,0;y(:,1:2)];
        dist = sqrt(differences(:,1).^2+differences(:,2).^2);
        dist(1)=[];

        idx = [];
        count = 1;
        while count < length(dist)
            id = min(find(cumsum(dist(count:end))>5));
            idx = [idx; id+count];
            count = id+count+1;
        end

        pt_x = y(idx,1);
        pt_y = y(idx,2);
        pt_psi  = y(idx,3);
        pt_v    = y(idx,4);
        pt_beta = y(idx,5);

        angles = pt_psi+pt_beta;
        
        sampled_data(i,j) = {[pt_x,pt_y,pt_psi,pt_v ,angles]};
    end
end

%%
figure;
scale_fact = 4;

for i = 1:N_mp/2
    for j = 1:N_mp/2
        traj_data = sampled_data{i,j};
        [final_idx,~] = size(traj_data);
        for ii = 1:final_idx
        draw_rectangle(traj_data(ii,1),traj_data(ii,2),traj_data(ii,3),c_map(i+j-1,:));
        quiver( traj_data(ii,1),traj_data(ii,2) , traj_data(ii,4)*cos(traj_data(ii,5))/scale_fact...
            , traj_data(ii,4)*sin(traj_data(ii,5))/scale_fact ,'k', 'LineWidth',1.5 , 'MaxHeadSize',  2 );
        alpha(.2) ;
        hold on;
        end
    end
end
axis equal