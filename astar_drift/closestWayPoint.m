function [closestWaypoint, closestLen] = closestWayPoint(STATE, PAR, wpInit)

    % STATE(1)=x
    % STATE(2)=y
    % PAR.OPT.waypoints
    x= 0;
    y= 0;
    x= STATE(1);
    y= STATE(2);

    map_x = PAR.OPT.waypoints(wpInit,1);
    map_y = PAR.OPT.waypoints(wpInit,2);
    len =size(PAR.OPT.waypoints,1);

    %init search direction
    closestLen = distance(x,y,map_x,map_y);
    closestWaypoint = wpInit;
    found=0;

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
    if lenPlus < closestLen
        dir = 1;
    else
        dir =-1;
    end

    wpnext = closestWaypoint;
  for i=1:len
    wpnext=wpnext+dir;
    if wpnext>len
        wpnext=wpnext-len;
    end
    if wpnext<1
        wpnext=wpnext+len;
    end      
    
    lenNext = distance(x,y,PAR.OPT.waypoints(wpnext,1),PAR.OPT.waypoints(wpnext,2));
    if lenNext < closestLen
        closestWaypoint=wpnext;
        closestLen = lenNext;
        found=1;
    elseif  lenNext > closestLen && closestLen < 10
        found=1;
        break
    end
  end
%debug
d=1;
% [state_glob, PAR] = loc2glob([PAR.OPT.waypoints(closestWaypoint,1), PAR.OPT.waypoints(closestWaypoint,2),0] , PAR);
% plot (state_glob(1),state_glob(2),'*')
% drawnow
