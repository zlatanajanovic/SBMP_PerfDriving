function [forbidden_tot, exp_array, PAR] = waypointcheck(mot_prim_array,exp_array, PAR)
forbidden_tot = false(PAR.OPT.exp_Nk,1);
wp=1;
len = PAR.OPT.wp_len;
for i=1:PAR.OPT.exp_Nk    
    [wp, dist]=closestWayPoint(mot_prim_array(i,:), PAR, wp);
    %Collision check
    if dist>PAR.OPT.width/2
        forbidden_tot(i)=1;    
    end
    
    
%     %progress
%     path_x= PAR.OPT.waypoints(wp,1)-PAR.OPT.waypoints(wp+1,1);
%     path_y= PAR.OPT.waypoints(wp,2)-PAR.OPT.waypoints(wp+1,2);
%     path_orient =atan2(path_y,path_x);
%     
%     progress = wp-PAR.OPT.root_wp;
%     if progress>0
%         progress=progress-len
%     end
%     alpha=0.2;
%     exp_array(i,1:2)=alpha*progress+ (1-alpha)*abs(path_orient-mot_prim_array(i,3)-mot_prim_array(i,4))*mot_prim_array(i,5);
%     
end
        