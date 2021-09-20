function [mot_prim_array, mot_prim_count, PAR] = mot_prim(mot_prim_array, STATE, INPUTS, PAR)    

%Function to return an array with motion primitives
% state FORMAT
% x, y, psi, beta, v, psidot


    %% Initialize
    mot_prim_array_temp = zeros(PAR.OPT.exp_Nk, 6)+1e5;
    mot_prim_array = zeros(PAR.OPT.exp_Nk, 6);
    %% Motion primitives
    %   simple discretisation ZOH, TODO improve
    %   State: x, y, psi, beta, v, psidot
    %   Inputs: delta, acc
    
    %%sample model  for initial development - extremely wrong!!!
    
%     %psi
%     mot_prim_array(:,3)= STATE(3)+ STATE(5)/PAR.VEH.len*PAR.OPT.exp_dt*INPUTS(:,2);
%     psi=(mot_prim_array(:,3)+STATE(3))/2;
%     %x  = xo + v cos t
%     mot_prim_array(:,1)= STATE(1) + STATE(5)*cos(psi)*PAR.OPT.exp_dt;
%     %y
%     mot_prim_array(:,2)= STATE(2)+ STATE(5)*sin(psi)*PAR.OPT.exp_dt;
%     
%     %beta=0
%     mot_prim_array(:,4)= STATE(4);
%     %v = v0+ acc t
%     mot_prim_array(:,5)= STATE(5)+ PAR.OPT.exp_dt* INPUTS(:,1);
%     %psidot
%     mot_prim_array(:,6);   



T_end = PAR.OPT.exp_dt;

x0 = STATE(1);
y0 = STATE(2);
psi0 = STATE(3);
v0 = STATE(5);
beta0 = STATE(4);
psidot0 = STATE(6);

%N_mp = 8;



v_delta = 1.5;

psidotD = .35;
betaD = .2;

psidotD = .4;
betaD = .3;

% if abs(psidot0) <= 0.2
%     betaD = .2;
% else
%     betaD = .4;
% end

Ts = 0.02;

pol_surface = 0;
if pol_surface 
    run('params_surface_poly');
    psidotMax = .5;
    betaMax   = .6;
else
    Beta_grid_str = load('surface_data_interp', 'Beta_grid');
    Beta_grid = Beta_grid_str.Beta_grid;
    Psidot_grid_str = load('surface_data_interp', 'Psidot_grid');
    Psidot_grid=Psidot_grid_str.Psidot_grid;
    delta_eval_grid_str = load('surface_data_interp', 'delta_eval_grid');
    delta_eval_grid =delta_eval_grid_str.delta_eval_grid;
    lambda_eval_grid_str = load('surface_data_interp', 'lambda_eval_grid');
    lambda_eval_grid = lambda_eval_grid_str.lambda_eval_grid;
    v_eval_grid_str = load('surface_data_interp', 'v_eval_grid');
    v_eval_grid = v_eval_grid_str.v_eval_grid;
    psidotMax = .75;
    betaMax   = .6;
end


% beta_eval = linspace(beta0-betaD,beta0+betaD,PAR.OPT.exp_Nstr); 
% psidot_eval = linspace(psidot0-psidotD,psidot0+psidotD,PAR.OPT.exp_Na); 


%
%
n_psidot=PAR.OPT.exp_Na;
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
n_beta = PAR.OPT.exp_Nstr;
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
%
%

% randomly_generated_samples=1;
% if randomly_generated_samples
%     generated_samples = mvnrnd([beta0; psidot0],[betaD/5, 0; 0,...
%     psidotD/5],2*PAR.OPT.exp_Nstr*PAR.OPT.exp_Na);
% 
%     beta_eval = [beta0 ; generated_samples(:,1)];
%     psidot_eval = [psidot0 ; generated_samples(:,2)];
% end


%eval(['state_0_' i '_' j ' = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_eval(' i '), lambda_r_eval(' j ')];']);
% 
% t_tot = cell(PAR.OPT.exp_Nstr,PAR.OPT.exp_Na);
% y_tot = cell(PAR.OPT.exp_Nstr,PAR.OPT.exp_Na);
count_positives = 0;
%positives = [];
k=1;
y_ij = [];

for i = 1:PAR.OPT.exp_Nstr %Beta
    for j = 1:PAR.OPT.exp_Na %psidot
        
% for ij = 1:length(generated_samples)
%       i = ij;
%       j = ij;
%        state_0 = [x0,y0,psi0,v0,beta0,psidot0, delta_eval(i), lambda_r_eval(j), delta_eval(i), lambda_r_eval(j)];
        
        inv_conditions_beta_psidot =  abs(beta_eval(i)) > betaMax ||  abs(psidot_eval(j))> psidotMax...
                || beta_eval(i) > 0.05 && psidot_eval(j)>=-0.1 || beta_eval(i)<-0.05 && psidot_eval(j)<=0.1;
            
	if ~inv_conditions_beta_psidot

        if abs(psidot_eval(j))>0.05
            if pol_surface
            [ v_eval , delta_eval, lambda_eval] = v_of_beta_psidot(sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)), params, params_delta, params_lambda);
            else
%               v_eval =      griddata(x_data_4fit,y_data_4fit,z_data_4fit ,sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
%               delta_eval =  griddata(x_data_4fit,y_data_4fit,z1_data_4fit,sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
%               lambda_eval = griddata(x_data_4fit,y_data_4fit,z2_data_4fit,sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
            
                v_eval = interp2(Beta_grid ,Psidot_grid ,v_eval_grid',sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
                delta_eval = interp2(Beta_grid ,Psidot_grid ,delta_eval_grid',sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
                lambda_eval = interp2(Beta_grid ,Psidot_grid ,lambda_eval_grid',sign(beta_eval(i)*psidot_eval(j))*abs(beta_eval(i)), abs(psidot_eval(j)));
               
            end
            
        else
 %           v_eval =v0;
            v_eval =v0;
            delta_eval = 0;
            lambda_eval = .2;
        end
        %abs(psidot0)>0.05 &&
        v_eval = v_eval*(1-sin(abs(psidot_eval(j)/2)));
        
        v_eval = v_eval*ones(1,PAR.OPT.exp_Nvel);
        
        if PAR.OPT.exp_Nvel==3
            v_eval = v_eval + [-v_delta, 0, v_delta];
        end
    
            
        for l =1:PAR.OPT.exp_Nvel
        
            invalidity_conditions =  isnan(v_eval(l)) || v_eval(l) > 42 ||  v_eval(l) < 7 || (  abs(v_eval(l)-v0) > 1 ) ;
            if invalidity_conditions 
                y_ij = 1e6*ones(1,6);

            else

                %positives = [positives; i,j, beta_eval(i),psidot_eval(j)];
                state0_surf = [x0,y0,psi0, v0, beta0 , psidot0 , v_eval(l), beta_eval(i), psidot_eval(j)];

                %ode45 solution
        %        [t_ij,y_ij] = ode45(@veh_model,[0,T_end],state_0);
                %analytic solution
                [t_ij,y_ij] = analytic_sol_surf(T_end,state0_surf,Ts);
        %         t_tot(i,j) = {t_ij};
        %         y_tot(i,j) = {[y_ij, delta_eval*ones(length(t_ij),1) , lambda_eval*ones(length(t_ij),1)]};
            end
        
                
        end
    else
            y_ij = 1e6*ones(1,6);

    end
        
            mot_prim_array_temp(k,:)= [y_ij(end,1),y_ij(end,2),y_ij(end,3),y_ij(end,5),y_ij(end,4),y_ij(end,6)];
            k=k+1;

        
    end
end

% if any( abs(positives(:,4))>0.2 ) 
%     stophere = 1;
%     idx_valid = abs(mot_prim_array(:,3)) <1e3;
%  %   mot_prim_array(idx_valid,:)
% end

% if count_positives<= 3
%     stophere = 1;
%     positives
% end

idx_valid = abs(mot_prim_array_temp(:,3)) <1e3;
mot_prim_count =sum(idx_valid);
mot_prim_array(1:mot_prim_count,:) = mot_prim_array_temp(idx_valid,:);
% 
% if any (abs(mot_prim_array(idx_valid,6))>.5)
%    % mot_prim_array(idx_valid,:)
% end
 