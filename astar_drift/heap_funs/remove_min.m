function [LIST, MAP] = remove_min(LIST, length, MAP, PAR)
% Length is already decreased

    if length >0
        LIST(1,:)=LIST(length+1,:);
%         LIST(length+1,:)=[]; %fixed size for coding
        LIST(length+1,:)=zeros(1,21);
        MAP(LIST(1,3), LIST(1,4), LIST(1,5), LIST(1,6), LIST(1,7), LIST(1,8))=1;
        [LIST, MAP] = down_heapify(LIST, length, MAP, 0, PAR);
    else
%         LIST(length+1,:)=[]; %fixed size for coding
        LIST(length+1,:)=zeros(1,21);
    end