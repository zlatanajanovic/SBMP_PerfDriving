 function [LIST, MAP] = up_heapify(LIST, length, MAP , i, PAR)
% % If i is a root, 
% if root(i-1) 
%     return 
% end
% 
% %if i is smaller than parent they should be exchanged
% par_i_min = parent(i-1);
% if LIST(i,1)<LIST(par_i_min+1,1)
%     %swap 
%     temp=LIST(i,:);
%     LIST(i,:)=LIST(par_i_min+1,:);
%     LIST(par_i_min+1,:)=temp;
%     
%     %exchange in map
%     tempMAP=MAP(LIST(i,3),LIST(i,2), LIST(i,4), LIST(i,5));
%     MAP(LIST(i,3),LIST(i,2), LIST(i,4), LIST(i,5))=MAP(LIST(par_i_min+1,3),LIST(par_i_min+1,2), LIST(par_i_min+1,4), LIST(par_i_min+1,5));
%     MAP(LIST(par_i_min+1,3),LIST(par_i_min+1,2), LIST(par_i_min+1,4), LIST(par_i_min+1,5))=tempMAP;
%     
%     [LIST, MAP] = up_heapify(LIST, length,MAP, par_i_min+1, PAR);
%     return
% end;
% return
% end
%     
    

% Gets Matlab index (staring from 1) heap index i-1
% Start up heapify
while root(i-1)==false
    %if i is smaller than parent they should be exchanged
    par_i_min = parent(i-1);
    if LIST(i,1)<LIST(par_i_min+1,1)
        %swap 
        temp = LIST(i,:);
        temp1 = LIST(par_i_min+1,:);
        LIST(i,:)=temp1;
        LIST(par_i_min+1,:)=temp;

        %exchange in map
        tempMAP=MAP(temp(3),temp(4), temp(5), temp(6), temp(7), temp(8));
        tempMAP1=MAP(temp1(3),temp1(4), temp1(5), temp1(6), temp1(7), temp1(8));
        MAP(temp(3),temp(4), temp(5), temp(6), temp(7), temp(8))=tempMAP1;
        MAP(temp1(3),temp1(4), temp1(5), temp1(6), temp1(7), temp1(8))=tempMAP;

        %recurse further
        i = par_i_min+1;
    else
        return
    end;
end
end

