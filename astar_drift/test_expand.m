%------------------------------------------------------------------------
%  1  | 2 | 3 | 4 | 5 |   6    |  7   |   8  |  9   | 10  |   11  |  12
%------------------------------------------------------------------------
% f(n)| d | v | t | l | parent | par  | par  | par  |g(n) | del_d | del_t 
% [kJ]|ind|ind|ind|ind|  d-in  | v-in | t-in | l-in |[kJ] |  (m)  |  (s) 
%------------------------------------------------------------------------

node_test(1,1) = 0;     % fn
node_test(1,2) = 1;     % kd
node_test(1,3) = 5;     % kv
node_test(1,4) = 2 ;     % kt
node_test(1,5) = 1;     % kl
node_test(1,6) = 1;     % parent d
node_test(1,7) = 1;     % parent v
node_test(1,8) = 1;     % parent t
node_test(1,9) = 1;     % parent l
node_test(1,10) = -1;    % g
node_test(1,11) = 1;    % delta d
node_test(1,12) = 0.1;  % delta t

% plot
figure
plot_tmax = 2;
plot_dmax = 15;
xlim ([0, plot_tmax]);
ylim ([0, plot_dmax]);
set(gca,'xtick',[0:PAR.OPT.dt:plot_tmax])
set(gca,'ytick',[0:PAR.OPT.ds:plot_dmax])
grid on 
grid minor
xlabel('time');
ylabel('distance');
hold on


[exp_array, exp_count] = expand_array( node_test , 101, 1 , MAP, roll(xNode), PAR, INPUT, DAT);
exp_array