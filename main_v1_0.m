%
% Main v1_0 Optimal motion planning - Agile driving usecase
% A-star 
% Zlatan Ajanovic, 2018-2019
% VIRTUAL VEHICLE Research Center

%%
clc;
close all;
clear
addpath('astar_drift');
addpath('astar_drift/heap_funs');
addpath('astar_drift/motion primitives');
addpath('tools');
%% Simulation options

%forward simulation loop
PAR.OPT.forwardsim = true;
% PAR.OPT.forwardsim = false;

%forward Astar replanning
PAR.OPT.forwardAstar = true;
% PAR.OPT.forwardAstar = false;


%Plot enable
PAR.SIM.plotenable = true;
% PAR.SIM.plotenable = false;

%Turn on/off animation of exploring
PAR.SIM.animatesearch = true;
% PAR.SIM.animatesearch = false;

%Turn on/off animation of expanding
PAR.SIM.animateexpand = true;
PAR.SIM.animateexpand = false;

%Turn on/off animation of final result
PAR.SIM.animatedrive = true;
% PAR.SIM.animatedrive = false;

%Turn on/off saving GIF
PAR.SIM.GIFactive = true;
PAR.SIM.GIFactive = false;

%Enable Compile
PAR.SIM.enable_compile = true;
PAR.SIM.enable_compile = false;

%% Plot parameters

N_mp = 8;
c_map = jet(N_mp);

%% ROAD Data
TMP.a_ellipse =  25;  %20
TMP.b_ellipse =   18; %15
TMP.t = linspace(0,2*pi,3000);
TMP.length_straight = 50;%400;

TMP.x_straight = (0:0.2:TMP.length_straight);
TMP.y_straight = TMP.b_ellipse*ones(length(TMP.x_straight),1)';

INPUT.ROAD.X = [TMP.x_straight(2:end-1) TMP.length_straight+TMP.a_ellipse*sin(TMP.t(1:1500)) fliplr(TMP.x_straight(2:end-1)) TMP.a_ellipse*sin(TMP.t(1501:end))];
INPUT.ROAD.Y = [TMP.y_straight(2:end-1) TMP.b_ellipse*cos(TMP.t(1:1500)) -TMP.y_straight(2:end-1) TMP.b_ellipse*cos(TMP.t(1501:end)) ];

load('x_y_path');
l_max = length(y_path_abs);
INPUT.ROAD.X = -y_path_abs'; 
INPUT.ROAD.Y =  x_path_abs'; 
INPUT.ROAD.X = [INPUT.ROAD.X INPUT.ROAD.X(1)];
INPUT.ROAD.Y = [INPUT.ROAD.Y INPUT.ROAD.Y(1)];

% INPUT.ROAD.X = interp1(1:length(INPUT.ROAD.X),INPUT.ROAD.X,linspace(1, length(INPUT.ROAD.X),l_max));
% INPUT.ROAD.Y = interp1(1:length(INPUT.ROAD.Y),INPUT.ROAD.Y,linspace(1, length(INPUT.ROAD.Y),l_max));

INPUT.ROAD.width=12;  % previously 10 
INPUT.ROAD.wp_len=size(INPUT.ROAD.X,2);
PAR.OPT.wp_len = size(INPUT.ROAD.X,2);

for i=1:INPUT.ROAD.wp_len-1
    TMP.ds(i)=sqrt((INPUT.ROAD.X(i+1)-INPUT.ROAD.X(i))^2+(INPUT.ROAD.Y(i+1)-INPUT.ROAD.Y(i))^2);
end
INPUT.ROAD.s =[0, cumsum( TMP.ds)];

PAR.OPT.width = INPUT.ROAD.width; 

PAR.OPT.waypoints(:,1)=INPUT.ROAD.X';
PAR.OPT.waypoints(:,2)=INPUT.ROAD.Y';

PAR.OPT.wp_s=INPUT.ROAD.s;
%% Vehicle parameters
% 
PAR.VEH.Pbn = 4500; %3485;%6e3;         %Power consumption of boardnet (W)
PAR.VEH.kt = 10.; %2           %Transmission ration
% Transmission efficiency ~0.8821
PAR.VEH.treff = 0.9;
PAR.VEH.rw = 0.3705; %0.3;          %Wheel radius (m)

PAR.VEH.M = 1500;          %Vehicle mass
PAR.VEH.fr = PAR.VEH.M*9.81*10e-3; %combined Rolling Resistance Coefficient
PAR.VEH.fg = PAR.VEH.M*9.81; % combined road gradient coefficient
% PAR.VEH.fair = 0.42;%
PAR.VEH.fair = 0.156;       %combined air drag coefficient

PAR.VEH.len = 7;       %vehicle length
%%  GRID
%  Drift
% PAR.OPT.dx = 4;        %[m]
% PAR.OPT.dy = 0.4;        %[m]
% PAR.OPT.dpsi = pi/11;    %[rad]
% PAR.OPT.dbeta = pi/16;   %[rad]
% PAR.OPT.dv = 1;       %[m/s]
% PAR.OPT.dpsidot = 0.2; %[rad/s]
% 
% PAR.OPT.Nx = 20; 
% PAR.OPT.Ny = 21;        %must be ODD!!!
% PAR.OPT.Npsi = 11;      %must be ODD!!!
% PAR.OPT.Nbeta = 7;     %must be ODD!!!
% PAR.OPT.Nv = 20;
% PAR.OPT.Npsidot = 7;   %must be ODD!!!


PAR.OPT.dx = 3.5;        %[m]
PAR.OPT.dy = 0.5;        %[m]
PAR.OPT.dpsi = pi/6;    %[rad]
PAR.OPT.dbeta = pi/11;   %[rad]
PAR.OPT.dv = 1.5;       %[m/s]
PAR.OPT.dpsidot = 0.2; %[rad/s]

PAR.OPT.Nx = 40; 
PAR.OPT.Ny = 17;        %must be ODD!!!
PAR.OPT.Npsi = 7;      %must be ODD!!!
PAR.OPT.Nbeta = 5;     %must be ODD!!!
PAR.OPT.Nv = 20; %13;
PAR.OPT.Npsidot = 7;   %must be ODD!!!

PAR.OPT.exp_dt = .8; %0.8;   %[s] expansion time step

%expand motion primitives
PAR.OPT.exp_Na = 11; %11; %6;        %[] number of acceleration disretization steps% Psidot
TMP.exp_amax = 2;      %[m/s2] max acceleration
TMP.exp_amin = -2;     %[m/s2] min acceleration
PAR.OPT.exp_Nstr =9; %9; %6;      %[] factor of steering delta%Beta
TMP.exp_strmax = 1;    %[m/s2] max acceleration
TMP.exp_strmin = -1;   %[m/s2] min acceleration
PAR.MOTPR.acc_arr = linspace(TMP.exp_amax, TMP.exp_amin, PAR.OPT.exp_Na);
PAR.MOTPR.str_arr = linspace(TMP.exp_strmax , TMP.exp_strmin, PAR.OPT.exp_Nstr);
[TMP.acc_inps, TMP.str_inps] = meshgrid(PAR.MOTPR.acc_arr, PAR.MOTPR.str_arr');
PAR.MOTPR.inputs = [TMP.acc_inps(:), TMP.str_inps(:)];

PAR.OPT.exp_Nvel =3;
PAR.OPT.exp_Nk = PAR.OPT.exp_Nvel*PAR.OPT.exp_Nstr*PAR.OPT.exp_Na;

PAR.OPT.exp_Na_bicycle = 5;
PAR.OPT.exp_Nstr_bicycle =5;

%% SIMULATION Initilization

STATE(1)=8;
STATE(2)= -295;
STATE(3)= pi/6;
STATE(4)= 0;
STATE(5)= 12;
STATE(6)= 0;

%Fig expl1
STATE(1)= 70;       %x
STATE(2)= -269;      %y
STATE(3)= pi/20; %psi
STATE(4)= 0;      %veta
STATE(5)=12;         %v
STATE(6)=0;     %psidot


%Fig exploration U turn
STATE(1)= -108.7;       %x
STATE(2)= -291;      %y
STATE(3)= pi*13/20; %psi
STATE(4)= 0;      %veta
STATE(5)=12;         %v
STATE(6)=0;     %psidot


% % crashing 
% STATE(1)= 142.9329;
% STATE(2)= -336.8455;
% STATE(3)= 1.5895;
% STATE(4)= 0.0062;
% STATE(5)= 14.2463;
% STATE(6)= -0.0547;


% STATE(1)=-32;
% STATE(2)= -305.4;
% STATE(3)= 0;
% STATE(4)= 0;
% STATE(5)= 12;
% STATE(6)= 0;

% STATE(1)=8;
% STATE(2)= -295;
% STATE(3)= pi/6;
% STATE(4)= 0;
% STATE(5)= 12;
% STATE(6)= 0;
% % 


% STATE(1)= 90;       %x
% STATE(2)= -268;      %y
% STATE(3)= 0; %psi
% STATE(4)= 0;      %veta
% STATE(5)=12;         %v
% STATE(6)=0;     %psidot
% 

% initial conditions circuit
% STATE(1)=-60;       %x
% STATE(2)= -328;      %y
% STATE(3)= 5/6*pi; %psi
% STATE(4)= 0;      %veta
% STATE(5)=12;         %v
% STATE(6)=0;     %psidot
% 
% 
% debug rettilineo
% STATE(1)=50;       %x
% STATE(2)= -273.8;      %y
% STATE(3)= 1/8*pi; %psi
% STATE(4)= 0;      %veta
% STATE(5)=13;         %v
% STATE(6)=0;     %psidot
% 
% 
% debug straight section
STATE(1)=0;       %x
STATE(2)= -297.25;      %y
STATE(3)= 1/6*pi; %psi
STATE(4)= 0;      %veta
STATE(5)=13;         %v
STATE(6)=0;     %psidot


STATE_HIST = [STATE];
TIME_HIST = [0];

%% required for bicycle model expansion
PAR.VEH.delta_str_MAX = 0.08;
PAR.VEH.lambda_MAX = 0.05;


% vehicle data
PAR.VEH.m =1294;
PAR.VEH.Cxr = 1.5*10^4;
PAR.VEH.Cyf = 3*10^4;
PAR.VEH.Cyr = 3*10^4;
PAR.VEH.lf = 1.283;
PAR.VEH.lr = 1.307;
PAR.VEH.Jz = 1294;

%% CONSTRAINTS
% %Maximum  acceleration
% PAR.OPT.CONSTR.accmax = inf;
PAR.OPT.CONSTR.accmax = 1;%3.5;
% PAR.OPT.CONSTR.accmin = -inf;
PAR.OPT.CONSTR.accmin = -3.5;
% 
% % Velocity limits
% INPUT.ROAD.vmaxarray = PAR.OPT.CONSTR.Vmax*ones(PAR.OPT.Nks);%0*PAR.OPT.s_array+15;
% INPUT.ROAD.vminarray = zeros(PAR.OPT.Nks);%0*PAR.OPT.s_array;
% % INPUT.ROAD.vminarray(10 : 90) = 14;
% 
% % Initial and final velocity
% PAR.OPT.CONSTR.kvfinal = round(15/PAR.OPT.dv)+1; %final speed
% PAR.OPT.CONSTR.kvinit  = round(15/PAR.OPT.dv)+1; %initial speed
% PAR.OPT.CONSTR.klfinal = 1; %final lane
% PAR.OPT.CONSTR.klinit = 2; %initial lane, without discretisation
% PAR.OPT.CONSTR.ksfinal = PAR.OPT.Nks;
% PAR.OPT.CONSTR.ksinit = 1; %initial postition ind
% 
% % Initial Ks for forward
% PAR.ksstart = 1; %Works only for 1 for now

%% Replanning parameters
PAR.OPT.T_rep = .8; % Replanning period for A*
%PAR.OPT.Nks_hor = 10; % Horizon length [n]
PAR.OPT.Nkt_hor =8; %8; % Horizon length [n]
PAR.OPT.Nnodes =600; % Maximum number of explored nodes [n]
% PAR.VEH.sensor_fview = 999; % Sensor field of view [m]
%% Forward sim parameters

PAR.SIM.dt_sim = 0.05; %[s] simulation time step
PAR.SIM.sim_time = 120; %0.9; %[s] simulation time

%% Variable placeholders
TMP.Nk_nodes = min(PAR.OPT.Nnodes, PAR.OPT.Nx* PAR.OPT.Ny* PAR.OPT.Npsi* PAR.OPT.Nbeta* PAR.OPT.Nv*PAR.OPT.Npsidot*PAR.OPT.Nkt_hor);
% TMP.Nk_nodes = 2000;

% DAT.ref_x_in = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_y_in = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_psi_in = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_beta_in = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_v_in = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_psidot_in = zeros(1,PAR.OPT.Nkt_hor);
% 
% DAT.ref_x_val = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_y_val = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_psi_val = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_beta_val = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_v_val = zeros(1,PAR.OPT.Nkt_hor);
% DAT.ref_psidot_val = zeros(1,PAR.OPT.Nkt_hor);

% DAT.time = zeros(1,PAR.OPT.Nkt_hor);


% DAT.CLOSED = zeros(TMP.Nk_nodes,21);
% DAT.OPEN = zeros(TMP.Nk_nodes,21);

%% Plot initial 

PAR.SIM.vehicle_lngth = 4.5;
PAR.SIM.vehicle_width = 2.4;


% TMP.PLT.pieceplots=[];
% TMP.PLT.plottemp=[];
% TMP.PLT.plottemp1=[];    
% TMP.PLT.d_t_egoplot =[];
% TMP.PLT.d_t_timelineplot=[];
% TMP.PLT.VEL.velocity_arr=[];
% TMP.PLT.VEL.velocity_text=[]; 

if PAR.SIM.plotenable
    figure('units','normalized','outerposition',[0 0 1 1])
    hold on
    [PAR, INPUT] = plt_init(PAR, INPUT); 
   
    TMP.PLT.ego_plot =draw_rectangle(STATE(1),STATE(2),STATE(3),c_map(1,:),PAR,1);
    TMP.PLT.optpath=[];
    TMP.PLT.rectanglearr =[];
    DAT.PLT.pieceplot_array  =[];
    drawnow;
    hold on
end


figcount = 0;
%% NODE
% state_0 = [x0,y0,psi0,v0,beta0,psidot0,delta0,lambda_r0, delta_asymp0, lambda_r_asymp0];

    %% FORWARD SIMULATION
    tic    
    if PAR.OPT.forwardsim
        cnt_sim_i =1;
        TMP.cnt_rep_i =1;
        SIM.time_elapsed=0;
        TMP.cumcost=0;        

        %Astar relevant        
        DAT.MAP = zeros(PAR.OPT.Nx, PAR.OPT.Ny, PAR.OPT.Npsi, PAR.OPT.Nbeta, PAR.OPT.Nv, PAR.OPT.Npsidot);
        %replanning
        SIM.time_replanned = SIM.time_elapsed - PAR.OPT.T_rep;
        %horizon
        
        DAT.OptPath = [];
        DAT.OptState = [];
        
        if PAR.SIM.GIFactive
            %GIF
            DAT.GIF.filename = 'drift.gif';
            drawnow
            DAT.GIF.frame = getframe(1);
            [DAT.GIF.im,DAT.GIF.map] = rgb2ind(DAT.GIF.frame.cdata,256,'nodither');
%             DAT.GIF.im(1,1,1,PAR.OPT.Nks-2) = 0; %sta je ovo
            DAT.GIF.k=0;
        end
%             delete(TMP.PLT.VEL.velocity_arr)
%             delete(TMP.PLT.VEL.velocity_text)  
        %% Simulation START
        while SIM.time_elapsed < PAR.SIM.sim_time       %simulation time

            
            %% Astar  planning START
            if PAR.OPT.forwardAstar
                %%replanning
                if SIM.time_elapsed>= SIM.time_replanned + PAR.OPT.T_rep;
                    SIM.time_replanned = SIM.time_elapsed;
    %                 DAT.CLOSED = zeros(TMP.Nk_nodes,21);
                    DAT.CLOSED=[];     
    %                 DAT.OPEN = zeros(TMP.Nk_nodes,21);
                    DAT.OPEN=[]; 
                    %% Transfomation of the road to vehicle coordinate frame
                    DAT.MAP = 0*DAT.MAP;
                    
                    % offroad obstacles                    
                    %%
                    DAT.OPEN_COUNT = 1;
                    DAT.CLOSED_COUNT = 0;
                   
                    %% Preparing data for replanning - Transforation
                    %TMP.est_cost = costest(PAR.OPT.s_array(TMP.ks_init),PAR.OPT.v_array(TMP.kv_init),INPUT.ROAD.elev(cnt_ks_sim),PAR.OPT.s_array(PAR.OPT.Nks),PAR.OPT.v_array(PAR.OPT.CONSTR.kvfinal),INPUT.ROAD.elev(PAR.OPT.Nks),INPUT.ROAD.roll(cnt_ks_sim), PAR);

                    
                    %waypoint transform                    
                    [STATE_REP, PAR_REP] =glob2locWP(STATE, PAR);              
                    
                    %State transformation
                    
                    %node transform
                    [TMP.current_node , PAR_REP] = insert_open(0,0,STATE_REP, PAR_REP, DAT);
                    DAT.OPEN(1,:) = TMP.current_node(1,:);
                    DAT.MAP(TMP.current_node(1,3), TMP.current_node(1,4),TMP.current_node(1,5), TMP.current_node(1,6),TMP.current_node(1,7),TMP.current_node(1,8)) = 1; % initial element in DAT.OPEN
                
                                                            
%                     %debug
%                     plot (PAR_REP.OPT.waypoints(:,1),PAR_REP.OPT.waypoints(:,2))
                    %%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % START A* Search
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
%                     current_state = node2state(DAT.OPEN(1,:), PAR);
%                     current_state_glob = loc2glob(current_state, PAR);
%                     cs_scatter = scatter(current_state_glob(1),current_state_glob(2),100,'k', 'filled');

                    tic
                    [PAR_REP, INPUT, DAT] = A_star(PAR_REP, INPUT, DAT);

                    DAT.ttoc=toc;
                    WRK.current_node = DAT.OPEN(1,:);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % END OF A* Search
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %% planning time statistics - variations
%                     if SIM.time_elapsed>=4 && SIM.time_elapsed<=20
%                         figcount = figcount+1;
%                         filename = ['frame_' num2str(figcount)];
%                         savefig(gcf,filename);
%                     end
                    
                    DAT.var_closed(cnt_sim_i, TMP.cnt_rep_i) = DAT.CLOSED_COUNT;
                    DAT.var_calc_time(cnt_sim_i, TMP.cnt_rep_i) = DAT.ttoc;
                    TMP.cnt_rep_i = TMP.cnt_rep_i+1;

                    %% Extract trajectory
                    TMP.tval= DAT.CLOSED(end,end);  %to be different than zero
                    TMP.node_ind = DAT.nFin;
                    
                    OPT_PATH=[];
                    OPT_STATE=[];
                     while( TMP.tval ~= 0 )
                            TMP.NODE = DAT.CLOSED(TMP.node_ind,:);
                            % node->continous
                            TMP.STATE_FR = node2state(TMP.NODE , PAR_REP);    
                            % Convert Frenet -> XY    
                            [TMP.STATE_XY, PAR_REP] = frenet2xy(TMP.STATE_FR, PAR_REP);
                            % Local -> Global
                            TMP.STATE_XY_GLOB = loc2glob(TMP.STATE_XY, PAR_REP);
                            %Add to path
                            OPT_PATH=[OPT_PATH;TMP.STATE_XY_GLOB];
                            OPT_STATE = [OPT_STATE; TMP.STATE_FR([4,5,6])];
                            %Parent node
                            TMP.tval = TMP.NODE(21);
                            if TMP.tval>0
                                TMP.node_ind=-DAT.MAP(TMP.NODE(9), TMP.NODE(10),TMP.NODE(11),TMP.NODE(12),TMP.NODE(13),TMP.NODE(14));
                            end
                     end
                     
                     OPT_PATH=OPT_PATH(end:-1:1,:);
                     OPT_STATE=OPT_STATE(end:-1:1,:);
                     
                     DAT.OptPath = OPT_PATH;
                     DAT.OptState =OPT_STATE;
                     %Plot
                     %Coment plot horizon
%                      if PAR.SIM.plotenable
%                         % Delete plot
%                         delete(TMP.PLT.optpath);
%                         delete(TMP.PLT.rectanglearr);
%                         %delete(DAT.PLT.pieceplot_array); \iavsd paper
%                         %comment
%                         hold on
%                         TMP.PLT.optpath=plot(OPT_PATH(:,1)',OPT_PATH(:,2)', 'g', 'linewidth', 2);
%                         for i=1:length(OPT_PATH(:,1))
%                             TMP.PLT.rectangle= draw_rectangle(OPT_PATH(i,1),OPT_PATH(i,2),OPT_PATH(i,3),c_map(1,:),PAR,.2);
%                             TMP.PLT.rectanglearr =[TMP.PLT.rectanglearr, TMP.PLT.rectangle];
%                         end
%                      end

                end
            end        
            %% Simulating ego & other vehicle

            SIM.time_elapsed = SIM.time_elapsed + PAR.SIM.dt_sim;
            SIM.time_plan = SIM.time_elapsed-SIM.time_replanned;
            TMP.plan_time = 0:PAR.OPT.exp_dt:(PAR.OPT.Nkt_hor-1)*PAR.OPT.exp_dt;

            % Ego vehicle movement simulation
            
            while length(OPT_PATH(:,1))~=length(TMP.plan_time)
                OPT_PATH = [OPT_PATH ; OPT_PATH(end,:)+ ...
                    (TMP.plan_time(2)-TMP.plan_time(1))*...
                    [ OPT_STATE(end,1)*cos(OPT_STATE(end,2)+OPT_PATH(end,3)),...
                    OPT_STATE(end,1)*sin(OPT_STATE(end,2) + OPT_PATH(end,3)), OPT_STATE(end,3), 0, 0, 0]];
                OPT_STATE = [OPT_STATE; OPT_STATE(end,:)];
                DAT.OptPath = OPT_PATH;
                DAT.OptState =OPT_STATE;
            end
            
            try
            STATE(1)=interp1(TMP.plan_time,OPT_PATH(:,1),SIM.time_plan);
            STATE(2)=interp1(TMP.plan_time,OPT_PATH(:,2),SIM.time_plan);
            STATE(3)=interp1(TMP.plan_time,OPT_PATH(:,3),SIM.time_plan);
            STATE(4)=interp1(TMP.plan_time,OPT_STATE(:,1),SIM.time_plan);
            STATE(5)=interp1(TMP.plan_time,OPT_STATE(:,2),SIM.time_plan);
            STATE(6)=interp1(TMP.plan_time,OPT_STATE(:,3),SIM.time_plan);
            catch err
                stophere=1;
            end
            
            STATE_HIST = [STATE_HIST, STATE];
            TIME_HIST = [TIME_HIST,SIM.time_elapsed];

            %% Animate drive
            if PAR.SIM.animatedrive && PAR.SIM.plotenable
               delete(TMP.PLT.ego_plot);

                TMP.PLT.ego_plot =draw_rectangle(STATE(1),STATE(2),STATE(3),c_map(1,:),PAR,1);
                drawnow 

                if PAR.SIM.GIFactive
                    % GIF
                    DAT.GIF.k=DAT.GIF.k+1;
                    DAT.GIF.frame = getframe(1);
                    DAT.GIF.im(:,:,1,DAT.GIF.k) = rgb2ind(DAT.GIF.frame.cdata,DAT.GIF.map,'nodither');
                end
                pause(PAR.SIM.dt_sim);

            end %end animate drive

        end
        DAT.Optcost = TMP.cumcost;    
        DAT.var_travel_time(cnt_sim_i) = SIM.time_elapsed;
        DAT.var_cost(cnt_sim_i) = DAT.Optcost;

    %% FORWARD SIMULATION END
end
%%
if PAR.SIM.GIFactive
    DAT.GIF.frame = getframe(1);
    DAT.GIF.im(:,:,1,DAT.GIF.k+1) = rgb2ind(DAT.GIF.frame.cdata,DAT.GIF.map,'nodither'); 
    DAT.GIF.subsample_n=2; % to make it faster
    DAT.GIF.imnew=DAT.GIF.im(:,:,1,1:DAT.GIF.subsample_n:end);
    DAT.GIF.imnew(:,:,1,end)=DAT.GIF.im(:,:,1,end);
    imwrite(DAT.GIF.imnew,DAT.GIF.map,DAT.GIF.filename,'DelayTime',PAR.SIM.dt_sim,'LoopCount',0)
end  
%%
DAT.ttoc = toc;
%%  Dispalying results on CONSOLE
%
TMP.disp = ['SIMULATION finished!!!'];
disp(TMP.disp)
TMP.disp = ['Total driving time(s) : ', num2str(SIM.time_elapsed)];
disp(TMP.disp)
TMP.disp = ['Total calculation time(s) : ', num2str(DAT.ttoc)];
disp(TMP.disp)
TMP.disp = ['Average forward calculation time(s) : ', num2str(DAT.ttoc/TMP.cnt_rep_i)];
disp(TMP.disp)