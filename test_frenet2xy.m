

%     PAR.OPT.waypoints=[-5:0.5:5;-5:0.5:5]';
%     PAR.OPT.waypoints(:,2)=0.2*PAR.OPT.waypoints(:,2).^3;    
    
    PAR.OPT.waypoints=[0:1:5;0:1:5]';
    PAR.OPT.waypoints(:,1)=0*PAR.OPT.waypoints(:,2);    
    
    PAR.OPT.wp_len =size(PAR.OPT.waypoints,1);
    
    for i=1:PAR.OPT.wp_len-1
        ds(i)=sqrt((PAR.OPT.waypoints(i+1,2)-PAR.OPT.waypoints(i,2))^2+(PAR.OPT.waypoints(i+1,1)-PAR.OPT.waypoints(i,1))^2);
    end
    PAR.OPT.wp_s=[0, cumsum(ds)];
    PAR.OPT.exp_Nk=1;
    
    wpInit =21;
    STATE(1)=1.5;
    STATE(2)=3;  
    STATE(3)=pi/4;
    
    [STATE_XY, PAR] = frenet2xy(STATE, PAR);
    STATE_XY
    %plot
    
    plot (PAR.OPT.waypoints(:,1),PAR.OPT.waypoints(:,2), 'o')
    xlim([-10, 10]);
    ylim([-10, 10]);
    hold on
    plot (STATE(1),STATE(2), '*')
    
%     grid on
%     plot (PAR.OPT.waypoints(wp,1),PAR.OPT.waypoints(wp,2), 'g.')