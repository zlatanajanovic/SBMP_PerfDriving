
%     k=[1:8;1:8]'
    PAR.OPT.waypoints=[-5:0.5:5;-5:0.5:5]';
    PAR.OPT.waypoints(:,2)=0.2*PAR.OPT.waypoints(:,2).^3;

    wpInit =5;
    STATE(1)=1.5;
    STATE(2)=1;  
    wp=closestWayPoint(STATE, PAR, wpInit)
    
    %plot
    
    plot (PAR.OPT.waypoints(:,1),PAR.OPT.waypoints(:,2), 'o')
    xlim([-10, 10]);
    ylim([-10, 10]);
    hold on
    plot (STATE(1),STATE(2), '*')
    
    grid on
    plot (PAR.OPT.waypoints(wp,1),PAR.OPT.waypoints(wp,2), 'g.')