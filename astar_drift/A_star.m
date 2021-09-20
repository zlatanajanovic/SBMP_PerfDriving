function [PAR, INPUT, DAT] = A_star(PAR, INPUT, DAT )

%% DAT.OPEN LIST FORMAT
%
% x, y, psi, beta, v, psidot
% costs (1-2)|state(3-8) | parent (9-14) | reman(15-20) | time
%--------------------------------------------------------------------------
%  1  |  2  | 3 | 4 |...| 8 | 9  |...|  14| 15 | ... | 20 |21
%--------------------------------------------------------------------------
% f(n)| g(n)|x1 |x2 |...|x6 |par1|...|par6|rem1| ... |rem6|
%           |ind|ind|...|ind|ind |...|ind |ind | ... |ind |ind
%--------------------------------------------------------------------------

%%
WRK=[];
WRK.current_node = DAT.OPEN(1,:);
WRK.map_ind = 0;
WRK.exp_count = 0;
WRK.exp_array = zeros(PAR.OPT.exp_Nk, 21); 
WRK.GIFsubsample =0;

DAT.nFin=0; %Final node
WRK.nFinLen=0; %length of final node
WRK.nFinVal=inf; %length of final node

if PAR.SIM.animatesearch 
    TMP.PLT.pieceplotdt_arr = [];
    TMP.PLT.pieceplotro_arr = [];
end
%% Start search   
while(WRK.current_node(21) <PAR.OPT.Nkt_hor) && (DAT.CLOSED_COUNT<PAR.OPT.Nnodes) && DAT.OPEN_COUNT~=0 
    

    
    %% Close openned node    
    %Move the Node to list DAT.CLOSED 
    %Close a node
    
    DAT.CLOSED_COUNT = DAT.CLOSED_COUNT + 1;
    DAT.CLOSED(DAT.CLOSED_COUNT,:) = WRK.current_node;
    %debug
    if DAT.OPEN(1,3)<1 || DAT.OPEN(1,4)<1 ||DAT.OPEN(1,5)<1 || DAT.OPEN(1,6)<1 ||DAT.OPEN(1,7)<1 ||DAT.OPEN(1,8)<1
            debugg=1; % no solution
    end
    DAT.MAP(DAT.OPEN(1,3), DAT.OPEN(1,4), DAT.OPEN(1,5), DAT.OPEN(1,6), DAT.OPEN(1,7), DAT.OPEN(1,8)) = -DAT.CLOSED_COUNT;
    DAT.OPEN_COUNT = DAT.OPEN_COUNT-1;
    if DAT.OPEN_COUNT>=0
        [DAT.OPEN , DAT.MAP] = remove_min(DAT.OPEN, DAT.OPEN_COUNT, DAT.MAP, PAR);
        %Set WRK.current_node(2) and WRK.current_node(3) to the node with minimum fn (top of the heap)
    else
%         DAT.OPEN = [];
%         WRK.MSG.h=msgbox('Sorry, No solution exists!','warn');
%         uiwait(WRK.MSG.h,5);
    end      
    
        % Anytime trajectory
    
    if WRK.current_node(21)>WRK.nFinLen || (WRK.current_node(21)==WRK.nFinLen && WRK.current_node(1)<WRK.nFinVal)
        DAT.nFin = DAT.CLOSED_COUNT;
        WRK.nFinLen = WRK.current_node(21);
        WRK.nFinVal = WRK.current_node(1);
    end % inFrames
    
    
%% Expand
      
%UPDATE LIST DAT.OPEN WITH THE SUCCESSOR NODES
%DAT.OPEN LIST FORMAT
% x, y, psi, beta, v, psidot
% costs (1-2)|state(3-8) | parent (9-14) | reman(15-20) | time
%--------------------------------------------------------------------------
%  1  |  2  | 3 | 4 |...| 8 | 9  |...|  14| 15 | ... | 20 |21
%--------------------------------------------------------------------------
% f(n)| g(n)|x1 |x2 |...|x6 |par1|...|par6|rem1| ... |rem6|
%           |ind|ind|...|ind|ind |...|ind |ind | ... |ind |ind
%--------------------------------------------------------------------------
%EXPANDED ARRAY FORMAT    

    if PAR.SIM.animatesearch  %comment when using simulink
        %highlight node being expanded
        curr_state = node2state(WRK.current_node, PAR);
        [curr_state, PAR] = frenet2xy(curr_state, PAR);
        glob_state = loc2glob(curr_state,PAR);
        scatt_expanded_node = plot(glob_state(1),glob_state(2),'ok','MarkerFaceColor', 'k','MarkerSize',10);
    end                         %comment when using simulink
    
    WRK.exp_array = zeros(PAR.OPT.exp_Nk, 21); 
    [WRK.exp_array, WRK.exp_count, DAT] = expand_array( WRK.exp_array, WRK.current_node, PAR, INPUT, DAT);
   
    if PAR.SIM.animatesearch    %comment when using simulink
        delete(scatt_expanded_node);
    end                         %comment when using simulink

    
    for cnt_exp=1:WRK.exp_count
        if WRK.exp_array(cnt_exp,1)~=inf
            WRK.map_ind = DAT.MAP(WRK.exp_array(cnt_exp,3), WRK.exp_array(cnt_exp,4), WRK.exp_array(cnt_exp,5),  WRK.exp_array(cnt_exp,6),  WRK.exp_array(cnt_exp,7),  WRK.exp_array(cnt_exp,8));
            
            if WRK.map_ind > 0 % If it is already explored, check if it is better
                if DAT.OPEN(WRK.map_ind,2) >  WRK.exp_array(cnt_exp,2) %if it is better, improve
                    [DAT.OPEN, DAT.MAP] = improve_val(DAT.OPEN, DAT.OPEN_COUNT, DAT.MAP , WRK.map_ind, WRK.exp_array(cnt_exp,:), PAR);           
                end
            elseif WRK.map_ind == 0 % If it is not explored insert new
                DAT.OPEN_COUNT = DAT.OPEN_COUNT+1;
                DAT.OPEN(DAT.OPEN_COUNT,:) = WRK.exp_array(cnt_exp,:);
                DAT.MAP(WRK.exp_array(cnt_exp,3), WRK.exp_array(cnt_exp,4), WRK.exp_array(cnt_exp,5), WRK.exp_array(cnt_exp,6), WRK.exp_array(cnt_exp,7), WRK.exp_array(cnt_exp,8)) = DAT.OPEN_COUNT;
                [DAT.OPEN, DAT.MAP] = up_heapify(DAT.OPEN, DAT.OPEN_COUNT, DAT.MAP, DAT.OPEN_COUNT, PAR);
            end
        end
    end %End of loop i=1:WRK.exp_count
    %% Animate search
    %
    
    if PAR.SIM.animatesearch 
        pause(.05);
        if PAR.SIM.GIFactive && mod(WRK.GIFsubsample, 1)==0
            % GIF
            DAT.GIF.k=DAT.GIF.k+1;
            DAT.GIF.frame = getframe(1);
            DAT.GIF.im(:,:,1,DAT.GIF.k) = rgb2ind(DAT.GIF.frame.cdata,DAT.GIF.map,'nodither'); 
        end
        WRK.GIFsubsample = WRK.GIFsubsample +1;
    end
    %% New current node
    WRK.current_node = DAT.OPEN(1,:);
    end %End of While Loop
%% End search
% 
%%

if PAR.SIM.animatesearch
    delete (TMP.PLT.pieceplotdt_arr);
    delete (TMP.PLT.pieceplotro_arr);
    delete (DAT.PLT.pieceplot_array);
end