function [STATE_XY, PAR] = frenet2xy(STATE_FR, PAR)

  %From Frenet to XY
  STATE_XY=STATE_FR;
  
  len = PAR.OPT.wp_len;
%   for i=1:PAR.OPT.exp_Nk
        s= STATE_FR(1);
        d= STATE_FR(2);
        theta_fr= STATE_FR(3);        
        
        %find coresponding waypoint
        [wp_s, wp_i, PAR] = findWP(s,PAR); 
        %next waypoint      
        wpp=wp_i+1;
        if wpp>len
            wpp=wpp-len;
        end
        %heading of the middle line - waypoints
        heading = atan2((PAR.OPT.waypoints(wpp,2)-PAR.OPT.waypoints(wp_i,2)),...
                        (PAR.OPT.waypoints(wpp,1)-PAR.OPT.waypoints(wp_i,1)));
        %x,y,s along the segment
        seg_s = s-wp_s;
        seg_x = PAR.OPT.waypoints(wp_i,1)+seg_s*cos(heading);
        seg_y = PAR.OPT.waypoints(wp_i,2)+seg_s*sin(heading);
        % x,y of the point
        perp_heading = heading-pi/2;
        x = seg_x + d*cos(perp_heading);
        y = seg_y + d*sin(perp_heading);
 
  
        STATE_XY(1:3)=[x,y, theta_fr+heading];
%   end
