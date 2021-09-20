%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM for eco-driving apprication
% A* search demo
% 14-11-2017
% Copyright 2017 VIRTUAL VEHICLE RESEARCH CENTER
% Author: Zlatan Ajanovic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Add path
addpath('heap_funs')  

%% Simulation options
%Plot enable
PAR.SIM.plotenable = true;
PAR.SIM.plotenable = false;

%Plot d-t enable
PAR.SIM.plotdtenable = true;
% PAR.SIM.plotdtenable = false;

%Turn on/off animation of exploring
PAR.SIM.animatesearch = true;
% PAR.SIM.animatesearch = false;

%Turn on/off animation of final result
PAR.SIM.animatedrive = true;
PAR.SIM.animatedrive = false;

%Turn on/off saving GIF
PAR.SIM.GIFactive = true;
PAR.SIM.GIFactive = false;

%Plot piecewise trajectories
PAR.SIM.plottrajspicewise = true;
PAR.SIM.plottrajspicewise = false;

%Enable collision with other traffic and overtaking possibility
PAR.OPT.collisionactive = true;
% PAR.OPT.collisionactive = false;

%Enable Traffic Lights as a constraint
PAR.OPT.TLactive = true;
% PAR.OPT.TLactive = false;

%% Vehicle parameters
% optimal velocity as balance between airdrag and Pbn
PAR.VEH.vopt = nthroot((PAR.VEH.Pbn/2/PAR.VEH.fair),3);

%% Other traffic participants

INPUT.recvehvel(1,1,:)  = [2  5 ];%18.5;           %Velocity of receding vehicle
INPUT.recvehdist(1,1,:) = [50 00 ];            %Initial distance between vehicles
INPUT.recvehlane(1,1,:) = [1 2 ];            %Initial distance between vehicles

INPUT.lnch_progress_t_l = 0;
INPUT.lnch_progress_t_r = 0;

% overtaking parameters
PAR.OPT.threshdist = 6; %threshold distance
PAR.OPT.threshvel = 3.0; %threshold velocity difference
PAR.OPT.lnchngcost = 50.0; %cost of changing lane
PAR.OPT.lnchngtime = 6.0; % time needed to change lane
%% Traffic lights
INPUT.TLdist  = [50  150 250];         %Distance of traffic light from vehicle initial TMP.position
INPUT.TLgreenon  = [10 60 110 160 210;
                    -20 30 80 130 180;
                    0 50 100 150 200];        %Timing of green light switching on
INPUT.TLredon = INPUT.TLgreenon-25;           %Timing of red light switching on !!! Red must start first

INPUT.recveh_Nk = min([length(INPUT.recvehvel),length(INPUT.recvehdist),length(INPUT.recvehlane)]);
INPUT.TL_Nk  = length(INPUT.TLdist);

%% Simulation par

PAR.SIM.vehicle_lngth = 6;
PAR.SIM.vehicle_width = 0.6;

%% Map
% DEFINE THE 2-D DAT.MAP ARRAY
% index <0  in CLOSED list closed; index>0  in OPEN list(heap) 0 not expl.
% ???? Obstacles ?????

PAR.OPT.Nkt_hor = 30;
PAR.OPT.dt = 3;
TMP.Nk_nodes = PAR.OPT.Nkv*PAR.OPT.Nks*PAR.OPT.Nkt_hor*((PAR.OPT.Nkl-1)*(PAR.OPT.Nkdl+1)+1);
% TODO Analysis needed how many in Open, how many in closed

%index of origin
PAR.OPT.ks_origin = 1; 
PAR.OPT.kt_origin = 1;

DAT.MAP = zeros(PAR.OPT.Nkv, PAR.OPT.Nks_hor, PAR.OPT.Nkt_hor, (PAR.OPT.Nkl-1)*(PAR.OPT.Nkdl+1)+1);


% Initial and final conditions

TMP.ks_init = 1;                     % Starting Position
TMP.kv_init = PAR.OPT.CONSTR.kvinit; % Starting Position
TMP.kt_init = 1; % Starting Position
TMP.kl_init = PAR.OPT.CONSTR.klinit; % Starting Position
TMP.del_d_init = 0;
TMP.del_t_init = 0;

%% Plot Initial 

if PAR.SIM.plotdtenable
    [TMP, PAR, INPUT] = plt_dt_init(TMP, PAR, INPUT);
end
if PAR.SIM.plotenable
    [TMP, PAR, INPUT] = plt_init(TMP, PAR, INPUT);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Node structure
%--------------------------------------------------------------------------
%  1  | 2 | 3 | 4 | 5 |  6  |  7  |  8  |  9   | 10  |  11 |  12 | 13 |  14
%--------------------------------------------------------------------------
% f(n)| d | v | t | l | par | par | par | par  |g(n) |del_d|del_t| t_l| t_r
% [kJ]|ind|ind|ind|ind| d-in| v-in| t-in| l-in |[kJ] | (m) | (s) | (s)| (s)
%--------------------------------------------------------------------------
%
%DAT.OPEN LIST STRUCTURE
%
DAT.OPEN=[];
% DAT.OPEN = zeros(TMP.Nk_nodes,12);

%DAT.CLOSED LIST STRUCTURE - Same as DAT.OPEN

DAT.CLOSED = zeros(TMP.Nk_nodes,12);
DAT.CLOSED=[];
% DAT.CLOSED_COUNT=0;

%cos h for INPUT.ROAD.roll resistance
INPUT.ROAD.roll = [PAR.VEH.fr*cumsum(cos(INPUT.ROAD.alfa)*PAR.OPT.ds,2, 'reverse'),0];

DAT.OPEN_COUNT = 1;
DAT.CLOSED_COUNT = 0;
TMP.path_cost = 0;
TMP.par_kd =0;
TMP.par_kv = 1;
TMP.par_kt = 1;
TMP.par_kl = 1;
TMP.est_cost = costest(PAR.OPT.s_array(TMP.ks_init),PAR.OPT.v_array(TMP.kv_init),INPUT.ROAD.elev(1),PAR.OPT.s_array(PAR.OPT.Nks),PAR.OPT.v_array(PAR.OPT.CONSTR.kvfinal),INPUT.ROAD.elev(PAR.OPT.Nks),INPUT.ROAD.roll(1), PAR);

TMP.current_node = insert_open(TMP.est_cost, TMP.ks_init, TMP.kv_init, TMP.kt_init, TMP.kl_init, TMP.par_kd, TMP.par_kv, TMP.par_kt, TMP.par_kl, TMP.path_cost, TMP.del_d_init, TMP.del_t_init, 0 , 0);
DAT.OPEN(1,:) = TMP.current_node;
DAT.MAP(TMP.current_node(3), TMP.current_node(2), 1, 1) = 1; % initial element in DAT.OPEN


if PAR.SIM.GIFactive
    %GIF
    TMP.GIF.filename = 'Astar.gif';
    drawnow
    TMP.GIF.frame = getframe(1);
    [TMP.GIF.im,TMP.GIF.map] = rgb2ind(TMP.GIF.frame.cdata,256,'nodither');
    TMP.GIF.im(1,1,1,PAR.OPT.Nks-2) = 0;
    TMP.GIF.k=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START A* Search
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic

[PAR, INPUT, DAT] = A_star(PAR, INPUT, DAT );

DAT.ttoc=toc;
WRK.current_node = DAT.OPEN(1,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% END OF A* Search
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

%% Display
TMP.disp=['OPTIMIZATION FINISHED with A*!!!'];
disp(TMP.disp)
TMP.disp=['For discretization PAR.OPT.ds=', num2str(PAR.OPT.ds),' and PAR.OPT.dv=', num2str(PAR.OPT.dv)];
disp(TMP.disp)
TMP.disp = ['Total driving time(s) : ', num2str((WRK.current_node(4)-1)*PAR.OPT.dt + WRK.current_node(12))];
disp(TMP.disp)
TMP.disp=['Total energy used(kJ):     ', num2str(WRK.current_node(10) * 1e-3)];
disp(TMP.disp)
TMP.disp=['Total calculation time(s): ', num2str(DAT.ttoc)];
disp(TMP.disp)

%% Extract trajectory
Optimal_path_ind=[];
Optimal_path_val=[];
Optimal_path_val(1,1)=(WRK.current_node(2)-1)*PAR.OPT.ds + WRK.current_node(11);
Optimal_path_val(1,2)=(WRK.current_node(3)-1)*PAR.OPT.dv;
Optimal_path_val(1,3)=(WRK.current_node(4)-1)*PAR.OPT.dt + WRK.current_node(12);
Optimal_path_val(1,4)=WRK.current_node(5);
% 
Optimal_path_ind(1,1)=WRK.current_node(2);
Optimal_path_ind(1,2)=WRK.current_node(3);
Optimal_path_ind(1,3)=WRK.current_node(4);
Optimal_path_ind(1,4)=WRK.current_node(5);

TMP.sind = WRK.current_node(6);
TMP.vind = WRK.current_node(7);
TMP.tind = WRK.current_node(8);
TMP.lind = WRK.current_node(9);

while( TMP.sind >= TMP.ks_init)
    TMP.node_ind = -DAT.MAP(TMP.vind, TMP.sind, TMP.tind, TMP.lind);
    TMP.vval = (DAT.CLOSED(TMP.node_ind,3)-1)*PAR.OPT.dv; %node_TMP.index returns the TMP.index of the node
    TMP.sval = (DAT.CLOSED(TMP.node_ind,2)-1)*PAR.OPT.ds + DAT.CLOSED(TMP.node_ind,11);
    TMP.tval = (DAT.CLOSED(TMP.node_ind,4)-1)*PAR.OPT.dt + DAT.CLOSED(TMP.node_ind,12);
    TMP.lval = DAT.CLOSED(TMP.node_ind,5);
    
    Optimal_path_ind = [Optimal_path_ind ; [TMP.sind TMP.vind TMP.tind TMP.lind]];
    Optimal_path_val = [Optimal_path_val ; [TMP.sval TMP.vval TMP.tval TMP.lval]];
    
    TMP.sind = DAT.CLOSED(TMP.node_ind,6);
    TMP.vind = DAT.CLOSED(TMP.node_ind,7);
    TMP.tind = DAT.CLOSED(TMP.node_ind,8);
    TMP.lind = DAT.CLOSED(TMP.node_ind,9);
    
end;

Optimal_path_ind = [Optimal_path_ind ; [TMP.sind TMP.vind TMP.tind TMP.lind]];
Optimal_path_val = [Optimal_path_val ; [TMP.sval TMP.vval TMP.tval TMP.lval]];
    
    
Optimal_path_val = Optimal_path_val(end:-1:1,:);
Optimal_path_val(:,4) = (Optimal_path_val(:,4)-1)/(PAR.OPT.Nkdl+1)+1;
Optimal_path_ind = Optimal_path_ind(end:-1:1,:);

%% Plot final

if PAR.SIM.plotenable
    % Plot reference velocity
    subplot(5,1,[1 2 3]);
    plot(Optimal_path_val(:,1),Optimal_path_val(:,2), 'g','LineWidth',4);
    % Plot reference lane
    subplot(5,1,4);  
    TMP.PLT.plot_ln_ch = stairs(Optimal_path_val(:,1), Optimal_path_val(:,4),'b','LineWidth',2);                  
end 
if PAR.SIM.plotdtenable
        % Plot reference velocity
    subplot(1,5,[3 4 5]);
    plot(Optimal_path_val(:,3), Optimal_path_val(:,1),'g','LineWidth',4);
    % Plot reference lane
    subplot(1,5,2);  
    TMP.PLT.plot_ln_ch = stairs( Optimal_path_val(:,4), Optimal_path_val(:,1),'b','LineWidth',2);  
    
    %lane change
    TMP.PLT.lanechange_l = Optimal_path_val(:,4);
    TMP.PLT.lanechange_d = Optimal_path_val(:,1);
    TMP.PLT.lanechange_d ( mod(TMP.PLT.lanechange_l,1)==0)=[];
    TMP.PLT.lanechange_l ( mod(TMP.PLT.lanechange_l,1)==0)=[];
    TMP.PLT.plot_ln_ch = plot(TMP.PLT.lanechange_l+0.5, TMP.PLT.lanechange_d, 'b.');
    TMP.PLT.plot_ln_ch = plot(TMP.PLT.lanechange_l-0.5, TMP.PLT.lanechange_d, 'b.');
        
end
%% Animate drive
if PAR.SIM.animatedrive
    delete(TMP.PLT.ego_plot)
    delete(TMP.PLT.vehicles_plot)
    delete(TMP.PLT.VEL.velocity_arr)
    delete(TMP.PLT.VEL.velocity_text)
    
    TMP.pos = [INPUT.recvehlane(:)-PAR.SIM.vehicle_width/2, INPUT.recvehdist(:)-PAR.SIM.vehicle_lngth/2, PAR.SIM.vehicle_width*ones(INPUT.recveh_Nk,1), PAR.SIM.vehicle_lngth*ones(INPUT.recveh_Nk,1)];
    TMP.pos_ego = [ Optimal_path_val(1,4)-PAR.SIM.vehicle_width/2, PAR.OPT.s_array(TMP.ks_init)+TMP.del_d_init-PAR.SIM.vehicle_lngth/2, PAR.SIM.vehicle_width, PAR.SIM.vehicle_lngth];
    TMP.SIM.dt_ani = 1/5;
    TMP.SIM.lane_past = Optimal_path_val(1,4);
    TMP.SIM.lanechangerate = 0;
    for cnt_drv_ani_t = 0:TMP.SIM.dt_ani:Optimal_path_val(end,3)
        for cnt_i=1:INPUT.recveh_Nk
            TMP.PLT.vehicles_plot(cnt_i) = rectangle('Position',TMP.pos(cnt_i,:),'Curvature',0.3);
            TMP.PLT.vehicles_plot(cnt_i).FaceColor = [TMP.PLT.colors(cnt_i,:) 1];
            TMP.PLT.vehicles_plot(cnt_i).EdgeColor = [0 0 0 1];
        end
        TMP.pos = TMP.pos + [zeros(INPUT.recveh_Nk,1) , INPUT.recvehvel(:)*TMP.SIM.dt_ani, zeros(INPUT.recveh_Nk,1), zeros(INPUT.recveh_Nk,1)] ;
    
        %ego vehicle        
        TMP.SIM.lane_now = interp1(Optimal_path_val(2:end,3),Optimal_path_val(2:end,4),cnt_drv_ani_t, 'next');
        TMP.SIM.distance_now = interp1(Optimal_path_val(2:end,3),Optimal_path_val(2:end,1),cnt_drv_ani_t);
        
        if mod(TMP.SIM.lane_now,1)==0 %if now in middle of lane
            TMP.SIM.lanechangerate = 0;
            TMP.pos_ego(1) = TMP.SIM.lane_now-0.3;
        else
            if TMP.SIM.lanechangerate ==0 % if lane change just started
                TMP.SIM.lanechangerate = 2*(TMP.SIM.lane_now - TMP.SIM.lane_past)/(PAR.OPT.lnchngtime);
            end
        end
        TMP.SIM.lane_past = TMP.SIM.lane_now;
        
        TMP.pos_ego(1) =  TMP.pos_ego(1)+TMP.SIM.lanechangerate*TMP.SIM.dt_ani;
        TMP.pos_ego(2) =  TMP.SIM.distance_now-3;
        
        TMP.PLT.ego_plot = rectangle('Position',TMP.pos_ego,'Curvature',0.3);
        TMP.PLT.ego_plot.FaceColor = [1 1 1 1];
        TMP.PLT.ego_plot.EdgeColor = [0 0 0 1];
        subplot(1,5,[3 4 5]);
        TMP.PLT.d_t_egoplot = plot (cnt_drv_ani_t, TMP.SIM.distance_now, 'ro');
        TMP.PLT.d_t_timelineplot = plot ([cnt_drv_ani_t, cnt_drv_ani_t], [PAR.OPT.s_array(1),PAR.OPT.s_array(end)], '--k');
        subplot(1,5,2);      
        
        
        % Animate TL switching
        for cnt_i=1:INPUT.TL_Nk           
            TMP.index_tl = find(INPUT.TLredon(cnt_i,:) <= cnt_drv_ani_t, 1, 'last');            
            if INPUT.TLredon(cnt_i,TMP.index_tl)<=cnt_drv_ani_t & cnt_drv_ani_t<INPUT.TLredon(cnt_i,TMP.index_tl)+3 % yellow light                
                TMP.PLT.TLroad_r_plot(cnt_i).FaceColor = [0.5 0 0 1];            
                TMP.PLT.TLroad_y_plot(cnt_i).FaceColor = [1 1 0 1];            
                TMP.PLT.TLroad_g_plot(cnt_i).FaceColor = [0 0.5 0 1];                
            elseif INPUT.TLredon(cnt_i,TMP.index_tl)+3<=cnt_drv_ani_t& cnt_drv_ani_t<INPUT.TLgreenon(cnt_i,TMP.index_tl) % red light                
                TMP.PLT.TLroad_r_plot(cnt_i).FaceColor = [1 0 0 1];            
                TMP.PLT.TLroad_y_plot(cnt_i).FaceColor = [0.35 0.35 0 1];            
                TMP.PLT.TLroad_g_plot(cnt_i).FaceColor = [0 0.5 0 1];                
            elseif INPUT.TLgreenon(cnt_i,TMP.index_tl)<= cnt_drv_ani_t % green light
                TMP.PLT.TLroad_r_plot(cnt_i).FaceColor = [0.5 0 0 1];            
                TMP.PLT.TLroad_y_plot(cnt_i).FaceColor = [0.35 0.35 0 1];            
                TMP.PLT.TLroad_g_plot(cnt_i).FaceColor = [0 1 0 1];
            end                  
        end        
        
        drawnow 
        if PAR.SIM.GIFactive
            % GIF
            TMP.GIF.k=TMP.GIF.k+1;
            TMP.GIF.frame = getframe(1);
            TMP.GIF.im(:,:,1,TMP.GIF.k) = rgb2ind(TMP.GIF.frame.cdata,TMP.GIF.map,'nodither');
        end
        
%         pause(TMP.SIM.dt_ani);
        delete(TMP.PLT.vehicles_plot);
        delete(TMP.PLT.ego_plot);
        delete(TMP.PLT.d_t_egoplot);
        delete(TMP.PLT.d_t_timelineplot);
    end       
    
end
if PAR.SIM.GIFactive
    TMP.GIF.frame = getframe(1);
    TMP.GIF.im(:,:,1,TMP.GIF.k+1) = rgb2ind(TMP.GIF.frame.cdata,TMP.GIF.map,'nodither'); 
    TMP.GIF.subsample_n=5; % to make it faster
    TMP.GIF.imnew=TMP.GIF.im(:,:,1,1:TMP.GIF.subsample_n:end);
    TMP.GIF.imnew(:,:,1,end)=TMP.GIF.im(:,:,1,end);
    imwrite(TMP.GIF.imnew,TMP.GIF.map,TMP.GIF.filename,'DelayTime',0,'LoopCount',1)
end
%% Plot picewise
if PAR.SIM.plottrajspicewise
    subplot(5,1,[1 2 3]);
    %DAT.OPEN nodes
    for cnt_i=1:size(DAT.OPEN,1)
        plot([DAT.OPEN(cnt_i,6)-1, DAT.OPEN(cnt_i,2)-1]*PAR.OPT.ds, [DAT.OPEN(cnt_i,7)-1, DAT.OPEN(cnt_i,3)-1]*PAR.OPT.dv, 'r')
    end
    %DAT.CLOSED nodes
    for cnt_i=1:size(DAT.CLOSED,1)
        plot([DAT.CLOSED(cnt_i,6)-1, DAT.CLOSED(cnt_i,2)-1]*PAR.OPT.ds, [DAT.CLOSED(cnt_i,7)-1, DAT.CLOSED(cnt_i,3)-1]*PAR.OPT.dv, 'b')
    end
end
