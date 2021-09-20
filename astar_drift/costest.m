function costest = costest( x1, v1, elev1, x2, v2, elev2, roll, PAR)
% This function estimates cost for transition between two states 
%
%   difference in kinnetic and potential energy
%   x1    - start position [m]
%   v1    - vector of initial velocities [m/s]
%   elev1 - initial elevation
%   x2    - end position [m]
%   v2    - vector of final velocities [m/s]
%   elev2 - final elevation
%   
%
%

s_tra = (x2-x1);

%% Difference in Kinetic and potential energy

costest=PAR.VEH.M*(v2.^2-v1.^2)/2 + PAR.VEH.fg*(elev2-elev1); 
% costest=0;

%% Cost of roll resistance

costest = costest + roll;

%% Efficiency
 
effic = 0.9;
costest_pos = costest;
costest_pos(costest_pos<0)=0;
costest_neg = -costest;
costest_neg(costest_neg<0)=0;

costest = costest_pos./effic - costest_neg.*effic;

%% Cost of air drag and boardnet power

% acceleration determining to reach vopt
a1=zeros(size(v1));
a2=zeros(size(v2));
a1(v1>PAR.VEH.vopt)=PAR.OPT.CONSTR.accmin;
a1(v1<PAR.VEH.vopt)=PAR.OPT.CONSTR.accmax;
a2(v2>PAR.VEH.vopt)=PAR.OPT.CONSTR.accmax;
a2(v2<PAR.VEH.vopt)=PAR.OPT.CONSTR.accmin;
% distance

s1=(PAR.VEH.vopt^2-v1.^2)./(2*a1);
s2=(v2.^2-PAR.VEH.vopt^2)./(2*a2);
s_sum = s1 + s2;

costestair=zeros(size(v1));

% In case there is enough distance do speed up / slow down to optimal vel

t11 = (PAR.VEH.vopt-v1)./a1;
cost11 = PAR.VEH.fair*(v1.^3.*t11  + 1.5*v1.*a1.*t11.^2+ a1.^3.*t11.^4/4) + PAR.VEH.Pbn.*t11;
t12 = (v2-PAR.VEH.vopt)./a2;
cost12 = PAR.VEH.fair*(PAR.VEH.vopt.^3.*t12  + 1.5*PAR.VEH.vopt.*a2.*t12.^2+ a2.^3.*t12.^4/4) + PAR.VEH.Pbn.*t12;

costestair_temp = (s_tra-s_sum).*(PAR.VEH.fair*PAR.VEH.vopt.^2+PAR.VEH.Pbn/PAR.VEH.vopt) + cost11 + cost12;
costestair(s_sum <= s_tra) = costestair_temp(s_sum<=s_tra);

%In case there is not enough distance

%Meeting velocity
vx=sqrt(abs((2*a1.*a2.*s_tra + a2.*v1.^2 - a1.*v2.^2)./(a2-a1)));

t21=(vx-v1)./a1;
cost21 = PAR.VEH.fair*(v1.^3.*t21  + 1.5*v1.*a1.*t21.^2+ a1.^3.*t21.^4/4) + PAR.VEH.Pbn.*t21;   
t22=(v2-vx)./a2;
cost22 = PAR.VEH.fair*(vx.^3.*t22  + 1.5*vx.*a2.*t22.^2+ a2.^3.*t22.^4/4) + PAR.VEH.Pbn.*t22;

costestair_temp = cost21+cost22;
%If initial and final velocity are on the same side of opt velocity
% costestair(s>=s_tra)= costestair_temp(s>=s_tra); %test

costestair(s_sum>s_tra & a1.*a2 <0)= costestair_temp(s_sum>=s_tra& a1.*a2<0);
%If initial and final velocity are on the opisite sides of opt velocity
%(impossible acceleration)
costestair(s_sum>s_tra & a1.*a2>0)= NaN;

%Add to overall cost
costest = costest + costestair;

% simpler version
% costest=costest + s_tra*(PAR.VEH.fair*PAR.VEH.vopt.^2+PAR.VEH.Pbn/PAR.VEH.vopt);
