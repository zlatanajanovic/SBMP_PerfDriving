function [v, delta ,lambda ] = v_of_beta_psidot(beta, psidot, params, params_delta, params_lambda)

 p00 = params(1);
 p01 = params(2);
 p02 = params(3);
 p03 = params(4); 
 p04 = params(5); 
 p05 = params(6);
 
 p10 = params(7); 
 p11 = params(8); 
 p12 = params(9); 
 p13 = params(10);
 p14 = params(11); 
 
 p20 = params(12); 
 p21 = params(13); 
 p22 = params(14); 
 p23 = params(15); 
 
 p30 = params(16); 
 p31 = params(17); 
 p32 = params(18); 

 p40 = params(19); 
 p41 = params(20); 

 p50 = params(21); 

 
p00_delta = params_delta(1);
p01_delta = params_delta(2);
p10_delta = params_delta(3); 
p11_delta = params_delta(4); 
p20_delta = params_delta(5); 
p21_delta = params_delta(6); 
p30_delta = params_delta(7); 
p31_delta = params_delta(8); 
p40_delta = params_delta(9); 
p41_delta = params_delta(10); 
p50_delta = params_delta(11);

p00_lambda  = params_lambda(1);
p01_lambda  = params_lambda(2);
p10_lambda  = params_lambda(3);
p11_lambda  = params_lambda(4);
p20_lambda  = params_lambda(5);

v = p00 + p10*beta + p01*psidot + p20*beta^2 + p11*beta*psidot + p02*psidot^2 +...
                    p30*beta^3 + p21*beta^2*psidot + p12*beta*psidot^2 + p03*psidot^3 +...
                    p40*beta^4 + p31*beta^3*psidot + p22*beta^2*psidot^2 + p13*beta*psidot^3 +...
                    p04*psidot^4 + p50*beta^5 + p41*beta^4*psidot + p32*beta^3*psidot^2 +...
                    p23*beta^2*psidot^3 + p14*beta*psidot^4 + p05*psidot^5;
                
delta = p00_delta + p10_delta*beta + p01_delta*psidot + p20_delta*beta^2 + p11_delta*beta*psidot + ...
                    p30_delta*beta^3 + p21_delta*beta^2*psidot + ...
                    p40_delta*beta^4 + p31_delta*beta^3*psidot + p50_delta*beta^5 + p41_delta*beta^4*psidot;                
                
lambda = p00_lambda + p10_lambda*beta + p01_lambda*psidot + p20_lambda*beta^2 + p11_lambda*beta*psidot;
                
% partial_beta = 5*p50*beta^4 + 4*p41*beta^3*psidot + 4*p40*beta^3 + 3*p32*beta^2*psidot^2 +...
%     3*p31*beta^2*psidot + 3*p30*beta^2 + 2*p23*beta*psidot^3 + 2*p22*beta*psidot^2 + 2*p21*beta*psidot +...
%     2*p20*beta + p14*psidot^4 + p13*psidot^3 + p12*psidot^2 + p11*psidot + p10;
% 
% partial_psidot = p41*beta^4 + 2*p32*beta^3*psidot + p31*beta^3 + 3*p23*beta^2*psidot^2 +...
%     2*p22*beta^2*psidot + p21*beta^2 + 4*p14*beta*psidot^3 + 3*p13*beta*psidot^2 + 2*p12*beta*psidot +...
%     p11*beta + 5*p05*psidot^4 + 4*p04*psidot^3 + 3*p03*psidot^2 + 2*p02*psidot + p01;
