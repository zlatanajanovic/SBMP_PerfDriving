function [nodes, PAR] = state2node( parnode, states, states_count, PAR)%, DAT)
    nodes=zeros(states_count,21);

    [ind1, rem1] = discr(states(:,1), PAR.OPT.dx);      % x
    [ind2, rem2] = discr(states(:,2), PAR.OPT.dy);      % y
    [ind3, rem3] = discr(states(:,3), PAR.OPT.dpsi);    % psi
    [ind4, rem4] = discr(states(:,4), PAR.OPT.dbeta);   % beta
    [ind5, rem5] = discr(states(:,5), PAR.OPT.dv);      % v
    [ind6, rem6] = discr(states(:,6), PAR.OPT.dpsidot); % psidot
    

    nodes(:,3) = ind1;% x
    nodes(:,4) = ind2+(PAR.OPT.Ny-1)/2;% y -negative
    nodes(:,5) = ind3+(PAR.OPT.Npsi-1)/2;% psi -negative
    nodes(:,6) = ind4+(PAR.OPT.Nbeta-1)/2;% beta -negative
    nodes(:,7) = ind5;% v
    nodes(:,8) = ind6+(PAR.OPT.Npsidot-1)/2;% psidot -negative
    nodes(:,9) = parnode(3);% par x
    nodes(:,10) = parnode(4);% par y
    nodes(:,11) = parnode(5);% par psi
    nodes(:,12) = parnode(6);% par beta
    nodes(:,13) = parnode(7);% par v
    nodes(:,14) = parnode(8);% par psidot
    nodes(:,15) = rem1;% rem x
    nodes(:,16) = rem2;% rem y
    nodes(:,17) = rem3;% rem psi
    nodes(:,18) = rem4;% rem beta
    nodes(:,19) = rem5;% rem v
    nodes(:,20) = rem6;% rem psidot
    nodes(:,21) = parnode(21)+1;% time cout
    
    
    
    if ~isempty(states)

        
        nodes(:,2) = parnode(2) + states(:,5)*PAR.OPT.exp_dt;  %distance covered
    
        parent_state = node2state(parnode, PAR);  %node2state returns in xy form
%            PAR_temp = PAR;
%            PAR_temp.OPT.origin = PAR_temp.OPT.origin-parent_state(1:3);
%         [parent_state_glob, PAR] = loc2glob(parent_state, PAR_temp);
        
prev_path_weight = 1;

    prev_path_cost = zeros(length(nodes(:,2)),1);
%     if ~isempty(DAT.OptPath) && (PAR.OPT.exp_dt == PAR.OPT.T_rep) && (parnode(21)>0) && (parnode(21)+3<=PAR.OPT.Nkt_hor)
%         OPT_PATH = DAT.OptPath ;
%         OPT_STATE = DAT.OptState;
%         [states_xy, PAR] = frenet2xy(states, PAR);
%         states_xy(:,2) = -states_xy(:,2);
%         [state_glob, PAR] = loc2glob(states_xy, PAR);
%         [r_length,~] = size(state_glob);
%         try
%         diff_ = (state_glob(:,1:3)-repmat(OPT_PATH(parnode(21)+3,:),r_length,1));
%         catch err
%             stophere=1;
%         end
%         
%         prev_path_cost = prev_path_weight*normalize_vector( sum((diff_*diag([.1 .5 1])).^2,2) );
%     end
    

    corner_cutting_weight=  1 * nodes(:,2) ./ (states(:,1)+1);
    
    
   
    frenet_angle_dist_weight=0;
    if (parnode(21)+1)>=PAR.OPT.Nkt_hor-5
        %frenet_angle_dist_weight= normalize_vector(((states(:,3)+parent_state(3))+parent_state(4)).^2);
        frenet_angle_dist_weight= 10*normalize_vector((states(:,3)+states(:,4)).^2);
    end

    
%     dist_weight=0;
%     if (parnode(21)+1)>=PAR.OPT.Nkt_hor-1
%         dist_weight=(parnode(21)+1)/PAR.OPT.Nkt_hor;
%     end
 dist_weight=(parnode(21)+1)/PAR.OPT.Nkt_hor;   
 d_weight = 2;
 
 %try
     
angle_cost_on_straight = 100*max(0,(1-10*abs(states(:,7)))).*normalize_vector(states(:,3).^2);
     
cost2go_weight = .5;     
dyn_states_coeffs = 1*[2 1 5 .5 1 1]; 
dyn_states_vect = [  100*abs(states(:,7)).*normalize_vector( abs(states(:,4)) - .6 ).^2 , ...           %don't mind a high beta
                + normalize_vector(states(:,4)-parent_state(4)).^2 ,...       %beta variation
                + normalize_vector(states(:,5)-parent_state(5)).^2, ...    %v variation
                + normalize_vector(states(:,6)-parent_state(6)).^2,  ...    %psidot variation
                + normalize_vector(sign(states(:,4))+sign(states(6))).^2, ...
                + normalize_vector(states(:,3)-parent_state(3)).^2 ] ;
dyn_states_cost = (dyn_states_coeffs*dyn_states_vect')';
            

avoid_edges = 50*normalize_vector(max(0,states(:,2).^4-3.5^4));
keep_centre = 20*normalize_vector(states(:,2).^2);
centre_position = avoid_edges + keep_centre;

%  catch err
%      stophere=1;
%  end
%500*(abs(states(idx_valid,4))-abs(.6)).^2, + 500*(states(idx_valid,4)-parnode(6)).^2, +1*(states(idx_valid,5)-parnode(7)).^2,+ 10*(states(idx_valid,6)-parnode(8)).^2
    
%try
   nodes(:,1) = -dist_weight*d_weight*states(:,1) -cost2go_weight*cost2go(parnode(21)+1, states, PAR) + corner_cutting_weight +...
            dist_weight*normalize_vector(states(:,2).^2) + dyn_states_cost+dist_weight*frenet_angle_dist_weight+...
            centre_position + prev_path_cost + angle_cost_on_straight;% f(n) %TODO costest
% catch err
%   stophere=1;
% end


%  if (parnode(21)+1)>=PAR.OPT.Nkt_hor || (parnode(21)+1)>=PAR.OPT.Nkt_hor-2 && any(nodes(:,1)<100)
% %     display(-5*dist_weight*states(:,1)); 
% %     display(-1*cost2go(parnode(21)+1, states, PAR));
% %     display(10*corner_cutting_weight );
% % 	display(dist_weight*normalize_vector(states(:,2).^2));
% %     display(20*dyn_states_cost);
% %     display(dist_weight*500*frenet_angle_dist_weight);
% %     display(normalize_vector(max(0,states(:,2).^4-4.5^4)));
% %     display(prev_path_weight);
% cost_matrix = [nodes(:,1),-dist_weight*d_weight*states(:,1), -cost2go_weight*cost2go(parnode(21)+1, states, PAR), + corner_cutting_weight ,+...
%             dist_weight*normalize_vector(states(:,2).^2), dyn_states_cost, dist_weight*frenet_angle_dist_weight,+...
%             centre_position,prev_path_cost, + angle_cost_on_straight];
% 
%     [r_len,~] = size(dyn_states_vect);
%     cost_dyn_states = [dyn_states_cost, (dyn_states_vect.*repmat(dyn_states_coeffs,r_len,1))];
% 
% %         figure;
% %         barplot1 = bar(cost_matrix);
% %         legend('total','distance','cost2go','corner cutting','distance from centre','dyn states','frenet angle', 'centre position', 'previous path','angle cost on straight');
% %         close(gcf)
% %         
% %          figure;
% %         barplot2 = bar(cost_dyn_states);
% %         legend('dyn states','beta drift','variation beta','variation v','variation psidot','beta/psidot opposition','variation psi');
% %         close(gcf)
%  end
 %      -  states(idx_valid,1),  dist_weight*1*states(idx_valid,2).^2,    0.2*dyn_states_cost(idx_valid) 
%      - states(idx_valid,1) + dist_weight*1*states(idx_valid,2).^2 + 0.2*dyn_states_cost(idx_valid) 
         
%    nodes(:,2) = - states(:,1);% g(n)
%      nodes(:,2) = parnode(2)+ dist_weight*.1*states(:,2).^2 + 10*dyn_states_cost;
%      nodes(:,1) = -.1*states(:,1)+nodes(:,2);

    end
    
    %curvature - <0, right turn, >0 left; Radius = 1/Curvature
    %states(:,7)