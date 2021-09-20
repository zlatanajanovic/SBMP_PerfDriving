function [LIST, MAP] = down_heapify(LIST, length, MAP, ind, PAR)
% Gets heap index


%% Start down heapify
    while ind<=length
        % If i is a leaf, 
        if (right(ind)>=length) && (left(ind)>=length)
            return 
        end
        left_ind_p = 1+ left(ind);
        right_ind_p = 1+right(ind);
        % If i has one child
        if eq(right(ind),length) %one_child(length,ind)
            if LIST(1+ind,1) > LIST(left_ind_p,1)
                temp=LIST(1+ind,:);
                LIST(1+ind,:)=LIST(left_ind_p,:);
                LIST(left_ind_p,:)=temp;

                %exchange in map
                tempMAP = MAP(LIST(1+ind,3),LIST(1+ind,4), LIST(1+ind,5), LIST(1+ind,6), LIST(1+ind,7), LIST(1+ind,8));
                MAP(LIST(1+ind,3),LIST(1+ind,4), LIST(1+ind,5),  LIST(1+ind,6),  LIST(1+ind,7),  LIST(1+ind,8))=MAP(LIST(left_ind_p,3),LIST(left_ind_p,4), LIST(left_ind_p,5), LIST(left_ind_p,6), LIST(left_ind_p,7), LIST(left_ind_p,8));
                MAP(LIST(left_ind_p,3),LIST(left_ind_p,4), LIST(left_ind_p,5), LIST(left_ind_p,6), LIST(left_ind_p,7), LIST(left_ind_p,8))=tempMAP;
            end
            return
        end
        % If i has two children
        % check heap property
        if min(LIST(left_ind_p,1), LIST(right_ind_p,1))>=LIST(1+ind,1)
            return
        end
        %if it fails, see which child is smaller and swap i's value into that child
        %afterwards, recurse into that child, whcih might violate

        if LIST(left_ind_p,1)<LIST(right_ind_p)
            %swap into left child (in LIST and MAP
            temp_old=LIST(1+ind,:);
            tempMAP=MAP(temp_old(3), temp_old(4), temp_old(5), temp_old(6), temp_old(7), temp_old(8));

            temp_new = LIST(left_ind_p,:);
            LIST(1+ind,:)= temp_new;
            MAP(temp_old(3),temp_old(4), temp_old(5), temp_old(6), temp_old(7), temp_old(8))=MAP(temp_new(3),temp_new(4), temp_new(5), temp_new(6), temp_new(7), temp_new(8));

            LIST(left_ind_p,:)=temp_old;    
            MAP(temp_new(3),temp_new(4), temp_new(5), temp_new(6), temp_new(7), temp_new(8))=tempMAP;
            
            %recurse further        
            ind = left_ind_p -1;
        else
            %swap into right child (in LIST and MAP

            temp = LIST(1+ind,:);
            LIST(1+ind,:)=LIST(right_ind_p,:);
            LIST(right_ind_p,:)= temp;

            %exchange in map
            tempMAP=MAP(LIST(1+ind,3),LIST(1+ind,4), LIST(1+ind,5), LIST(1+ind,6), LIST(1+ind,7), LIST(1+ind,8));
            MAP(LIST(1+ind,3),LIST(1+ind,4), LIST(1+ind,5), LIST(1+ind,6), LIST(1+ind,7), LIST(1+ind,8))=MAP(LIST(right_ind_p,3),LIST(right_ind_p,4), LIST(right_ind_p,5), LIST(right_ind_p,6), LIST(right_ind_p,7), LIST(right_ind_p,8));
            MAP(LIST(right_ind_p,3),LIST(right_ind_p,4), LIST(right_ind_p,5), LIST(right_ind_p,6), LIST(right_ind_p,7), LIST(right_ind_p,8))=tempMAP;

            %recurse further        
            ind = right_ind_p - 1;
        end
    end
end

