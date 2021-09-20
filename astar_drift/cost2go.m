function c2g = cost2go(current_t, states, PAR)  

T= (PAR.OPT.Nkt_hor-current_t)* PAR.OPT.exp_dt;
v_max= (PAR.OPT.Nv-1)*PAR.OPT.dv;

v0=cos(abs(states(:,3))-abs(states(:,4))).*states(:,5);
%v0 = states(:,5);

T_acc=(v_max-v0)./PAR.OPT.CONSTR.accmax;


c2g = v0.*T_acc+PAR.OPT.CONSTR.accmax*T_acc.^2/2;
c2g2=v_max.*T-(v_max-v0).*T_acc/2;
c2g(T_acc>T) = c2g2(T_acc>T);