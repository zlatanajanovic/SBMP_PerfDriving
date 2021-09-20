function [STATE, PAR] = glob2locWP(STATE, PAR)

%% New Origin
[wp, closestLen] =closestWayPoint([STATE(1),STATE(2)], PAR, 1);

wp=wp-1; %to be sure s is positive
if wp<1
    wp=wp+PAR.OPT.wp_len;
end      

PAR.OPT.root_wp = wp;
PAR.OPT.root_s  = PAR.OPT.wp_s(wp);
wpp=wp+1;

if wp==PAR.OPT.wp_len
    wpp=1;
end

  %heading of the middle line - waypoints
  heading = atan2((PAR.OPT.waypoints(wpp,2)-PAR.OPT.waypoints(wp,2)),...
                  (PAR.OPT.waypoints(wpp,1)-PAR.OPT.waypoints(wp,1)));
              
PAR.OPT.origin = [PAR.OPT.waypoints(wp,1),PAR.OPT.waypoints(wp,2), heading];


%% Adapting waypoints 

% s                    
PAR.OPT.wp_s=PAR.OPT.wp_s-PAR.OPT.root_s;
PAR.OPT.wp_s(PAR.OPT.wp_s<0)=PAR.OPT.wp_s(PAR.OPT.wp_s<0)+PAR.OPT.root_s+PAR.OPT.wp_s(end);


% Planar transformation : rotation and translation of waypoints

mRot = [cos(-heading) -sin(-heading);sin(-heading) cos(-heading)];

PAR.OPT.waypoints(:,1:2) = PAR.OPT.waypoints(:,1:2) + repmat([-PAR.OPT.origin(1), -PAR.OPT.origin(2)], size(PAR.OPT.waypoints,1),1);
PAR.OPT.waypoints(:,1:2) = (mRot*PAR.OPT.waypoints(:,1:2)')';


TMP.state=[ STATE(1), STATE(2)];
TMP.state =  TMP.state+ [-PAR.OPT.origin(1), -PAR.OPT.origin(2)];
TMP.state = (mRot*TMP.state')';

STATE(1)= TMP.state(1);
STATE(2)= TMP.state(2);
STATE(3) = STATE(3)-heading;

while STATE(3)<-pi
    STATE(3)=STATE(3)+2*pi;
end
while STATE(3)>=pi
    STATE(3)=STATE(3)-2*pi;
end



