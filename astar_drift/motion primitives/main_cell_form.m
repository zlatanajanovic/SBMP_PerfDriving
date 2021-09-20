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
delta0 = 0.2;
lambda_r0 = 0.1;


% delta_asymp0 = .5;
% lambda_r_asymp0 = .4;



N_mp = 6;
c_map = jet(N_mp);

deltaD = .2;
lambda_rD = .06;

delta_eval = linspace(delta0-deltaD,delta0+deltaD,N_mp/2); 
lambda_r_eval = linspace(lambda_r0-lambda_rD,lambda_r0+lambda_rD,N_mp/2); 


%eval(['state_0_' i '_' j ' = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_eval(' i '), lambda_r_eval(' j ')];']);

t_tot = cell(N_mp/2);
y_tot = cell(N_mp/2);
sampled_data = cell(N_mp/2);

Ts = 0.02;

tic
for i = 1:N_mp/2
    for j = 1:N_mp/2
        
        state_0 = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_eval(i), lambda_r_eval(j)];
        %ode45 solution
        %[t_ij,y_ij] = ode45(@veh_model,[0,T_end],state_0);
        %analytic solution
        [t_ij,y_ij] = analytic_sol(T_end,state_0,Ts);
        t_tot(i,j) = {t_ij};
        y_tot(i,j) = {y_ij};
        
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

%%
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