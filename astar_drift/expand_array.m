function [exp_array, exp_count, DAT] = expand_array( exp_array, node, PAR, INPUT, DAT)    

%Function to return an array with expanded nodes and count
%This function takes a node and returns the expanded list
%of successors, with the calculated fn values.
%OPEN LIST FORMAT
% x, y, psi, beta, v, psidot
% costs (1-2)|state(3-8) | parent (9-14) | reman(15-20) | time
%--------------------------------------------------------------------------
%  1  |  2  | 3 | 4 |...| 8 | 9  |...|  14| 15 | ... | 20 |21
%--------------------------------------------------------------------------
% f(n)| g(n)|x1 |x2 |...|x6 |par1|...|par6|rem1| ... |rem6|
%           |ind|ind|...|ind|ind |...|ind |ind | ... |ind |ind
%--------------------------------------------------------------------------
% Note: - d rounded to previous for dt cutting trajectories
%       - All nodes explored - improvement possible using amax


    %% Initial State calculation : node->continous
    STATE_FR = node2state(node,PAR);
    
    %% Convert Frenet -> XY
    
    [STATE_XY, PAR] = frenet2xy(STATE_FR, PAR);
    
    %% Motion primitives
    %   simple discretisation ZOH, TODO improve
    %   State: x, y, psi, beta, v, psidot
    %   Inputs: delta, acc
    
    mot_prim_array = zeros(PAR.OPT.exp_Nk, 6);

  %  
    
%	[mot_prim_array_drift , mot_prim_count_drift] 
    
    mot_prim_count_bicycle = 0;
    mot_prim_array_bicycle = zeros(PAR.OPT.exp_Na_bicycle*PAR.OPT.exp_Nstr_bicycle, 6);

   
  %    if abs(STATE_XY(4))>=0.04 &&  abs(STATE_XY(6))>=0.25
        [mot_prim_array_drift , mot_prim_count_drift]= mot_prim(mot_prim_array, STATE_XY, PAR.MOTPR.inputs , PAR);
  %    else
  if  abs(STATE_XY(4))<=0.1 &&  abs(STATE_XY(6))<=0.4
        [mot_prim_array_bicycle , mot_prim_count_bicycle] = mot_prim_bicycle(mot_prim_array, STATE_XY, PAR.MOTPR.inputs , PAR);
  end
      
        if mot_prim_count_drift+mot_prim_count_bicycle >= PAR.OPT.exp_Nk
          stophere=1;
        end
  
      mot_prim_array = [
          mot_prim_array_drift(1:mot_prim_count_drift,:);
          mot_prim_array_bicycle(1:max(1,mot_prim_count_bicycle),:);          
          zeros(PAR.OPT.exp_Nk-(mot_prim_count_drift+max(1,mot_prim_count_bicycle)),6)   ];
      
      mot_prim_count = mot_prim_count_drift+mot_prim_count_bicycle;
      
  

      
   %           mot_prim_array = mot_prim_array_drift;
%           mot_prim_count = mot_prim_count_drift;

      
      
%     idx_valid = abs(mot_prim_array(:,3)) <1e3;
%     if any (abs(mot_prim_array(idx_valid,6))>.5)
%         mot_prim_array(idx_valid,:);
%     end
    
    %[len_r,len_c] = size(mot_prim_array);
    %PAR.OPT.exp_Nk = len_r;
%     PAR.OPT.exp_Nk = mot_prim_count;
%     mot_prim_array(mot_prim_count+1:end,:)=[];
    %% Convert XY -> Frenet
    wpInit=PAR.OPT.root_wp;
    
    %idx_valid = abs(mot_prim_array(:,3))<1e5;
    %mot_prim_array(idx_valid,:)
    [mot_prim_sd_array] = xy2frenet(mot_prim_array, mot_prim_count, PAR, wpInit);
    
%     mot_prim_sd_array = mot_prim_sd_array( abs(mot_prim_sd_array(:,3)) <1e3 ,:);
%      [len_r,len_c] = size(mot_prim_sd_array);
%     PAR.OPT.exp_Nk = len_r;
    
    %% Fromatting expand: continous->node
%    exp_count =len_r; % PAR.OPT.exp_Nk;
    exp_count = PAR.OPT.exp_Nk;
    
 %   exp_array = zeros(len_r, 21);  %PAR.OPT.exp_Nk
    exp_array = zeros(PAR.OPT.exp_Nk, 21);  %PAR.OPT.exp_Nk
    
    exp_array = state2node( node, mot_prim_sd_array, PAR.OPT.exp_Nk, PAR);%, DAT);

    
    %% Collision check
    % return forbidden
    % is it on the road TODO
%    forbidden_tot = false(len_r,1);
      forbidden_tot = true(PAR.OPT.exp_Nk,1);
      forbidden_tot(1:mot_prim_count)= false;

%     [forbidden_tot, exp_array, PAR] = waypointcheck(mot_prim_array,exp_array,PAR);
    
    
    %% Remove out of the range 
    
    forbidden_tot = forbidden_tot | exp_array(:,3)<1;
    forbidden_tot = forbidden_tot | exp_array(:,4)<1;
    forbidden_tot = forbidden_tot | exp_array(:,5)<1;
    forbidden_tot = forbidden_tot | exp_array(:,6)<1;
    forbidden_tot = forbidden_tot | exp_array(:,7)<1;
    forbidden_tot = forbidden_tot | exp_array(:,8)<1;    
    
    forbidden_tot = forbidden_tot | exp_array(:,3)>PAR.OPT.Nx;    
    forbidden_tot = forbidden_tot | exp_array(:,4)>PAR.OPT.Ny;
    forbidden_tot = forbidden_tot | exp_array(:,5)>PAR.OPT.Npsi;
    forbidden_tot = forbidden_tot | exp_array(:,6)>PAR.OPT.Nbeta;
    forbidden_tot = forbidden_tot | exp_array(:,7)>PAR.OPT.Nv;
    forbidden_tot = forbidden_tot | exp_array(:,8)>PAR.OPT.Npsidot;
    
    %mean_ = -(forbidden_tot'-1)*exp_array(:,1)/(exp_count-sum(forbidden_tot(:)));
    
    %forbidden_tot = forbidden_tot | and(mean_>0,exp_array(:,1)>6/exp_array(1,end)*mean_);
    
    
    exp_count = exp_count-sum(forbidden_tot(:));
    exp_array(forbidden_tot,:)=[];
    
    exp_array(:,1) = exp_array(:,1) + 10*mean(exp_array(:,1))/(1+exp_count^3);
    
    
    if exp_count<= 2
        stophere = 1;
    end
    
    
    %% Plot
    
      if PAR.SIM.animatesearch         
        if PAR.SIM.plotenable
            
                %Closed node
                state_p_glob = loc2glob(STATE_XY, PAR);
                TMP.PLT.pieceplot = plot(state_p_glob(1), state_p_glob(2) ,'go','MarkerFaceColor', 'g');
                DAT.PLT.pieceplot_array = [DAT.PLT.pieceplot_array, TMP.PLT.pieceplot];
    %           % TODO add expansions
            if PAR.SIM.animateexpand
                state_glob = loc2glob(mot_prim_array, PAR);
                for i=1: PAR.OPT.exp_Nk %len_r
                    if forbidden_tot(i)==0
                        TMP.PLT.pieceplot = plot([state_p_glob(1), state_glob(i,1)], [state_p_glob(2), state_glob(i,2)] ,'r-'); 
                        DAT.PLT.pieceplot_array = [DAT.PLT.pieceplot_array, TMP.PLT.pieceplot];

                        TMP.PLT.pieceplot = plot(state_glob(i,1), state_glob(i,2) ,'ro'); 
                        DAT.PLT.pieceplot_array = [DAT.PLT.pieceplot_array, TMP.PLT.pieceplot];

                        drawnow;
%                         if mot_prim_sd_array(i,2)>5 || mot_prim_sd_array(i,2)<-5
%                             debugg;
%                         end
%                         if state_glob(i,1)<-30
%                             debugg;
%                         end
                    end
                end
            end
        end
      end    
end
 