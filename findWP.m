function [wp_s, wp_i, PAR] = findWP(s,PAR)
% Rerunt s value and index of the closest waypoing w , TODO : Binary search

[wp_ds, wp_i] = min (abs(PAR.OPT.wp_s-s));

if (PAR.OPT.wp_s(wp_i)-s)>0
    wp_i=wp_i-1;
    if wp_i==0
        wp_i=PAR.OPT.wp_len;
    end
end

wp_s=PAR.OPT.wp_s(wp_i);