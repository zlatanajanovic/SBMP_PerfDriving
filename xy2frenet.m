function [STATE_FR] = xy2frenet(STATE_XY, STATE_XY_len, PAR, wpInit)

%Transform from Cartesian x,y coordinates to Frenet s,d coordinates
STATE_FR=[STATE_XY, 0*STATE_XY(:,1)];
wp=wpInit;
len =PAR.OPT.wp_len;
for i=1:STATE_XY_len    
        [closestWaypoint, dist]=closestWayPoint(STATE_XY(i,:), PAR, wp);

        x = STATE_XY(i,1);
        y = STATE_XY(i,2);
        theta= STATE_XY(i,3);

        wpp=closestWaypoint+1;
        if wpp>len
            wpp=wpp-len;
        end
        wpm=closestWaypoint-1;
        if wpm<1
            wpm=wpm+len;
        end      

        lenPlus = distance(x,y,PAR.OPT.waypoints(wpp,1),PAR.OPT.waypoints(wpp,2));
        lenMinus = distance(x,y,PAR.OPT.waypoints(wpm,1),PAR.OPT.waypoints(wpm,2));

        if lenPlus<=lenMinus
            wp=closestWaypoint;
        else
            wp=wpm;
            wpp=closestWaypoint;
            wpm= wp-1;
            if wpm<1
                wpm=wpm+len;
            end  
        end

        n_x = PAR.OPT.waypoints(wpp,1) - PAR.OPT.waypoints(wp,1);
        n_y = PAR.OPT.waypoints(wpp,2) - PAR.OPT.waypoints(wp,2);
        x_x = x - PAR.OPT.waypoints(wp,1);
        x_y = y - PAR.OPT.waypoints(wp,2);

        % projection of x on n
        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
        proj_x = proj_norm*n_x;
        proj_y = proj_norm*n_y;

        frenet_d = distance(x_x,x_y,proj_x,proj_y);

        %see if d value is positive or negative
        %Sum over the edges, (x2 - x1)(y2 + y1). If the result is positive 
        %the curve is clockwise, if it's negative the curve is counter-clockwise. 
        %(The result is twice the enclosed area, with a +/- convention.)
        x0 = PAR.OPT.waypoints(wpm,1);
        y0 = PAR.OPT.waypoints(wpm,2);
        x1 = PAR.OPT.waypoints(wp,1);
        y1 = PAR.OPT.waypoints(wp,2);
        x2 = PAR.OPT.waypoints(wpp,1);
        y2 = PAR.OPT.waypoints(wpp,2);

        sum = (x2 - x1)*(y2 + y1);
        sum = (x  - x2)*(y  + y2)+sum;
        sum = (x1 - x) *(y1 + y) +sum;


%         d=(y2 - y1)*(x - x2) - (y - y2)*(x2 - x1);

%         dx1=x2-x1;
%         dy1=y2-y1;
%         dx2=x-x1;
%         dy2=y-y1;
        %   
        %angle = acos((dx1*dx2+dy1*dy2)/(sqrt(dx1^2+dy1^2)*sqrt(dx2^2+dy2^2)));

%         % with sample point
%         heading_wp = atan2(dx1,dy1);   
%         heading_dot = atan2(dx2,dy2);   
%         heading_rel=heading_dot-heading_wp;

        %
%         side = (x-x1)*(y2-y1)-(y-y1)*(x2-x1);
        if (sum<0)
            frenet_d = -1*frenet_d;
        end
        % calculate s value
        frenet_s = PAR.OPT.wp_s(wp)+distance(0,0,proj_x,proj_y);

        %heading of the middle line - waypoints
        heading = atan2((PAR.OPT.waypoints(wpp,2)-PAR.OPT.waypoints(wp,2)),...
                       (PAR.OPT.waypoints(wpp,1)-PAR.OPT.waypoints(wp,1)));

        STATE_FR(i,1:3)=[frenet_s,frenet_d, theta-heading ];
        
        %road curvature
        triangleArea= (x1-x0)*(y2-y0) - (y1-y0)*(x2-x0);
        sideLength1=distance(x0,y0,x1,y1);
        sideLength2=distance(x2,y2,x1,y1);
        sideLength3=distance(x0,y0,x2,y2);
        STATE_FR(i,7)=4*triangleArea/(sideLength1*sideLength2*sideLength3); 
  
end

