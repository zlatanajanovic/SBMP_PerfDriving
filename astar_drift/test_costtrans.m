cost=0;
% optray1=optray(end:-1:1);
% optray1=DAT.optrayval;
optray1=([Optimal_path(:,2)-1 ])*PAR.OPT.dv; %A*
for i=1:length(optray1)-1
    cost= cost + costtrans(optray1(i),optray1(i+1),INPUT.ROAD.ht(i),PAR);
end
cost

