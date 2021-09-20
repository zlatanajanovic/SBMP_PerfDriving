function [ind, rem] = discr(value, delta)    

% Function to return an index of discretisation
% accepts vectors
ind=floor((value)/delta)+1;

rem= value - (ind-1)*delta;
        
end
 