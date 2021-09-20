% Obstacle prediction uncertainty safety margin
clear


prov1=0.95;
prov2=0.95;
boja1=[0.45 0.05 0.45];
boja2=[0.45 0.15 0.45];
boja3=[0.85 0.85 0.5];


t_rep = 1.6;

dt=0.01;
%parameters
% vi = 30;
% si = 50;
% s_lim = 140;
% amin = -10;
% ds_err = 6;
% dv_err = 6;
% veh_len = 17;

vi = 30;
si = 50;
s_lim = 140;
amin = 0;
ds_err = 6;
dv_err = 0;
veh_len = 17;


safe_1 = 1 *(abs(amin)*t_rep^2/2 + dv_err*t_rep  +    ds_err);
safe_2 = 2*safe_1;


t=0:dt:3;
t_pred =0:dt:t_rep;

pos = si+vi*t+amin*t.^2/2;
vel = vi + amin*t;

% CoG movement
plt_cog_r=plot(t, pos, 'k' , 'linewidth', 2, 'Color', boja1)
hold on
%obstacle
plot_obs = fill([t , t(end:-1:1)], [pos-veh_len, pos(end:-1:1)+veh_len] , boja2)
set(plot_obs,'facealpha', 0.5)
uistack(plot_obs,'bottom');

k_start =1;
i=1

t_pred =0:dt:t(end);
t_pred1 =0:dt:t_rep;
t_pred2 =t_rep:dt:t(end);

v = vel(k_start)-(-1)^i*dv_err;

pos_predr = pos(k_start) +vel(k_start)*t_pred -(-1)^i*ds_err;

pos_pred = pos(k_start) +v*t_pred -(-1)^i*ds_err;
pos_pred1 = pos(k_start) +v*t_pred1-(-1)^i*ds_err;
pos_pred2 = pos(k_start) +v*t_pred2-(-1)^i*ds_err;

t_pred =(k_start-1)*dt+ t_pred;
t_pred1 =(k_start-1)*dt+ t_pred1;
t_pred2 =(k_start-1)*dt+ t_pred2;

% plot isprekidanu za T_rep
plot([t_pred(1),t_pred(1)], [0, pos_pred(1)+veh_len+safe_1+safe_2+5], '--k')

%CoG
plt_cog = plot(t_pred, pos_pred, 'k' , 'linewidth', 2)
%CoGr
plt_cogr = plot(t_pred, pos_predr, '--k' , 'linewidth', 1)
%CoG L
plt_cogL = plot(t_pred, pos_pred-veh_len, 'k' , 'linewidth', 1)
%     plot(t_pred, pos_pred+veh_len, '--b' , 'linewidth', 1)

plt_s = plot( [t_pred1 t_pred2], [pos_pred1-veh_len - safe_1, pos_pred2-veh_len-safe_1- safe_2], 'r' , 'linewidth', 4)
%     plt_s1 = plot(t_pred1, pos_pred1-veh_len-safe_1-safe_2, '--r' , 'linewidth', 1)
%     plt_s2 = plot(t_pred2, pos_pred2-veh_len-safe_1,  '--r' , 'linewidth', 1)


k_start = k_start + round( t_rep/dt);

%% second replanning
i=2
t_pred =0:dt:t(end);
t_pred1 =0:dt:t_rep;
t_pred2 =t_rep:dt:t(end);

v = vel(k_start)-(-1)^i*dv_err;

pos_predr = pos(k_start) +vel(k_start)*t_pred -(-1)^i*ds_err;

pos_pred = pos(k_start) +v*t_pred -(-1)^i*ds_err;
pos_pred1 = pos(k_start) +v*t_pred1-(-1)^i*ds_err;
pos_pred2 = pos(k_start) +v*t_pred2-(-1)^i*ds_err;

t_pred =(k_start-1)*dt+ t_pred;
t_pred1 =(k_start-1)*dt+ t_pred1;
t_pred2 =(k_start-1)*dt+ t_pred2;

% plot isprekidanu za T_rep
plot([t_pred(1),t_pred(1)], [0, pos_pred(1)+veh_len+safe_1+safe_2+5], '--k')

%CoG
plt_cog2 = plot(t_pred, pos_pred, '--k' , 'linewidth', 2)
%CoGr
% plt_cogr = plot(t_pred, pos_predr, '--k' , 'linewidth', 1)
%CoG L
% plt_cogL = plot(t_pred, pos_pred-veh_len, 'k' , 'linewidth', 1)
%     plot(t_pred, pos_pred+veh_len, '--b' , 'linewidth', 1)

plt_s2 = plot( [t_pred1 t_pred2], [pos_pred1-veh_len - safe_1, pos_pred2-veh_len-safe_1- safe_2], '--k' , 'linewidth', 3)
%     plt_s1 = plot(t_pred1, pos_pred1-veh_len-safe_1-safe_2, '--r' , 'linewidth', 1)
%     plt_s2 = plot(t_pred2, pos_pred2-veh_len-safe_1,  '--r' , 'linewidth', 1)





leg1=legend( [plt_cog_r, plot_obs plt_cog, plt_cogL, plt_s, plt_cog2, plt_s2], '$s_{k}(t)$', '$s_{k}(t)\pm L_S$', '$\hat{s}_{k1}(t)$', '$\hat{s}_{k1}(t)- L_S$', '$\underline{\hat{s}}_{k1}(t)$', '$\hat{s}_{k2}(t)$','$\underline{\hat{s}}_{k2}(t)$', 'Location','Best' ); 
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
pbaspect([2 1 1])
arrow3([-0.05 0 0], [3 0 0], 'k-',0.5,1.5)
axis tight
arrow3([0 -150 0], [0 s_lim 0],'k-',0.5,1.5 )
axis tight
set(findobj(gca,'type','Patch'),'LineStyle','none')

% camlight left; lighting gouraud; material metal
% set(gca,'Position',[0 0 1 1])
axis off

xlim([-0.4, 3])
ylim([-2, s_lim])

set(gca,'Position',[0 0 1 1])


txt1 = '$$t$$';
text(2.82,-6,txt1,'Interpreter','latex','FontSize',14 )

txt1 = '$$t_0$$';
text(-0.05,-6,txt1,'Interpreter','latex','FontSize',14 )

txt1 = '$$t_0+T_{rep}$$';
text(-0.15+t_rep,-6,txt1,'Interpreter','latex','FontSize',14 )

% 
% txt1 = '$$\Delta t_{exp}$$';
% text(dt*0.95,-0.08*ds,txt1,'Interpreter','latex','FontSize',14 )

txt1 = '$$s$$';
text(-0.1, s_lim-14,txt1,'Interpreter','latex','FontSize',14)

% txt1 = '$$\Delta s_{exp}$$';
% text(-0.13*dt,ds,txt1,'Interpreter','latex','FontSize',14)


%% Sm arrow
headWidth = 0.5;
headLength = 0.5;
k_start =round( t_rep/dt)+1;

arrow3([-0.04  si-veh_len+ds_err 0], [-0.04  si-veh_len+ds_err-safe_1 0],'k-',headWidth,headLength )
arrow3([-0.04  si-veh_len+ds_err-safe_1 0], [-0.04  si-veh_len+ds_err 0],'k-',headWidth,headLength )

txt1 = '$$s_M$$';
text(-0.17, si-veh_len-0.5*safe_1+ds_err,txt1,'Interpreter','latex','FontSize',12);

arrow3([-0.04+t_rep  pos(k_start)-veh_len 0], [-0.04+t_rep  pos(k_start)-veh_len-safe_2 0],'k-',headWidth,headLength )
arrow3([-0.04+t_rep  pos(k_start)-veh_len-safe_2 0], [-0.04+t_rep  pos(k_start)-veh_len 0],'k-',headWidth,headLength )

txt1 = '$$2 s_M$$';
text(-0.21+t_rep, pos(k_start)-veh_len-0.5*safe_2,txt1,'Interpreter','latex','FontSize',12);

%% S error
arrow3([-0.04  si+ds_err], [-0.04  si],'k-',headWidth,headLength )
arrow3([-0.04  si], [-0.04  si+ds_err],'k-',headWidth,headLength )
txt1 = '$$\Delta s_{max}$$';
text(-0.33, si+0.5*ds_err,txt1,'Interpreter','latex','FontSize',12);

% arrow3([t_rep  pos(k_start)-ds_err], [t_rep  pos(k_start)],'k-',headWidth,headLength )
% arrow3([t_rep  pos(k_start)], [t_rep  pos(k_start)-ds_err],'k-',headWidth,headLength )
% txt1 = '$$\Delta s_{max}$$';
% text(0.01+t_rep, pos(k_start)-0.5*ds_err,txt1,'Interpreter','latex','FontSize',10);

% %% v eror
% s_v_err_arr = [1.15*t_rep si+ds_err+vi*1.15*t_rep];
% k_v=1.35
% d_v_err_arr = [k_v*t_rep/cos(atan(vi))*cos(atan(vi+dv_err)), si+ds_err+k_v*t_rep/cos(atan(vi))*sin(atan(vi+dv_err))]
% 
% arrow3([s_v_err_arr 0], [d_v_err_arr 0],'k-',headWidth,headLength )
% arrow3([d_v_err_arr 0], [s_v_err_arr 0],'k-',headWidth,headLength )
% 
% txt1 = '$$\Delta v_{max}$$';
% poste=(s_v_err_arr + d_v_err_arr)./2 + [0.01 0];
% text( poste(1), poste(2) ,txt1,'Interpreter','latex','FontSize',12, 'Rotation',20);

