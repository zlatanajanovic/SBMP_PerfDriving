function costtrans = costtrans(v1, v2, dt, alpha, PAR)
% This function calculates cost for transition between two states
% assumptions: constant acceleration
%
% v1 - initial velocity
% v2 - final velocity
% dt - time of transition
% alpha - road slope angle
% PAR - Stucture containing vehicle parameters
%
% Note: Possible to use vectors

% Effmap = griddedInterpolant({PAR.EM.T_EM_col, PAR.EM.w_EM_row}, PAR.EM.eta_EM_map.', 'linear');

vavvec = (v2+v1)./2;
accvec = (v2-v1)./dt;
accvec(accvec > PAR.OPT.CONSTR.accmax) = inf;	%limitting positive acceleration
accvec(accvec < PAR.OPT.CONSTR.accmin) = inf; %limitting negative acceleration

F = PAR.VEH.M*accvec + PAR.VEH.fair*vavvec.^2 + PAR.VEH.fr*cos(alpha) + PAR.VEH.fg*sin(alpha);
Tmotvec = F.*(PAR.VEH.kt*PAR.VEH.rw);
wavvec = vavvec./(PAR.VEH.rw*PAR.VEH.kt);

Tmotvecpos = Tmotvec;
Tmotvecpos(Tmotvecpos<0) = 0;
Tmotvecneg = -Tmotvec;
Tmotvecneg(Tmotvecneg<0) = 0;
    
%Efficiency calculation
if false %PAR.OPT.useefficiencymap %false        
    %Efficiency map
    Tmotvecabs = Tmotvecpos+Tmotvecneg;
    eta_EM = WRK.Effmap(Tmotvecabs,wavvec);
    eta_EM(eta_EM<0.1) = 0.1;
    eta_EM(eta_EM>0.99) = 0.99;
    Pmotvec = WRK.Tmotvecpos./eta_EM.*wavvec - WRK.Tmotvecneg.*eta_EM.*wavvec;
else 
    %Fixed efficiency
%     Pmotvec = (F+0.2*abs(F)).*vavvec;
    Pmotvec = Tmotvecpos./0.85.*wavvec - Tmotvecneg.*0.85.*wavvec;
end

% Calculating costs for transitions
% COST= old cost + Propulsion power + Boardnet consumption
costtrans= Pmotvec.*dt + PAR.VEH.Pbn.*dt; 
costtrans(isnan(costtrans))= inf; 