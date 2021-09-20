function [LIST, MAP] = improve_val(LIST, length, MAP, i, newval, PAR)
    %Gets Matlab index
    %Close previous
    LIST(i,:)=newval;
    [LIST, MAP] = up_heapify(LIST, length, MAP, i,PAR);
end