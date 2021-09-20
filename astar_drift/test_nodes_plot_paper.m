v_in = [10];
s_in = [6 ];
t_in = [0.55];

ds_exp = 20;
dt_exp = 2.3;
dv = 4.5;
vmax = 25;
amax = 15;

ds_grid = 5.5;
dt_grid = 0.6;

t_limit = [-0.25, 3.25];
s_limit = [-2, 32.5];

origin = [0.1, 2];
vf=0:dv:vmax;

figure
hold on

pbaspect([1.8 1 1])

% plot grid
s_curr = origin(2);
t_curr = origin(1);

while s_curr < s_limit(2)*0.95    
    plot( [0 0.95*t_limit(2)],[s_curr s_curr],'Color',[0 0 0]+0.1, 'LineStyle', ':')    
    s_curr = s_curr + ds_grid;        
end
while t_curr < t_limit(2)*0.95    
    plot( [t_curr t_curr],[0 0.95*s_limit(2)],'Color',[0 0 0]+0.1, 'LineStyle', ':')    
    t_curr = t_curr + dt_grid;        
end
    
% grid arrows
% t
arrow3([2*dt_grid+origin(1) origin(2) 0], [3*dt_grid+origin(1) origin(2) 0], 'k-',0.5,1)
arrow3([3*dt_grid+origin(1) origin(2) 0], [2*dt_grid+origin(1) origin(2) 0], 'k-',0.5,1)

txt1 = '$$\Delta t_{grid}$$';
text(dt_grid*2.5+origin(1) , origin(2),txt1,'Interpreter','latex','FontSize',12, 'HorizontalAlignment' , 'center', 'VerticalAlignment' , 'bottom' )
% s

arrow3([0.1 5*ds_grid+origin(2) 0 ], [0.1 4*ds_grid+origin(2) 0], 'k-',0.5,1)
arrow3([0.1 4*ds_grid+origin(2) 0 ], [0.1 5*ds_grid+origin(2) 0], 'k-',0.5,1)

txt1 = '$$\Delta s_{grid}$$';
text(0.12,4.5*ds_grid+origin(2),txt1,'Interpreter','latex','FontSize',12, 'VerticalAlignment' , 'middle' )


%% plot coordinates
% xlabel('time')
% ylabel('distance')

% view(35,20)
arrow3([t_limit(1)+0.1 0 0], [t_limit(2) 0 0], 'k', 0.7, 2.1)
% axis tight
arrow3([0 s_limit(1) 0], [0 s_limit(2) 0] ,'k',  0.7, 2.1)
% axis tight
% set(findobj(gca,'type','Patch'),'LineStyle','none')

% camlight left; lighting gouraud; material metal
% set(gca,'Position',[0 0 1 1])
axis off

xlim(t_limit)
ylim(s_limit)


txt1 = '$$t$$';
text(t_limit(2)-0.2,-1,txt1,'Interpreter','latex','FontSize',12 )
txt1 = '$$s$$';
text(-0.1,s_limit(2)-2.5,txt1,'Interpreter','latex','FontSize',12)

set(gca,'Position',[0 0 1 1])

%% Plot expans
for cnt_i = 1: length(v_in)
    vi_t = v_in(cnt_i);
    si_t = s_in(cnt_i);
    ti_t = t_in(cnt_i);

    % trajektories which achieve vf at ds_exp
    t_ds_exp= 2*ds_exp./(vi_t+vf);
    a_ds_exp= (vf-vi_t)./t_ds_exp;
    % trajectories which achieve vf at dt_exp
    a_dt_exp= (vf-vi_t)./dt_exp;
    t_dt_exp= 0*a_dt_exp + dt_exp;
    % choosing of trajecotories
    vx=2*ds_exp/dt_exp-vi_t;
    a_ds_exp(vf<vx)=0;
    t_ds_exp(vf<vx)=0;
    a_dt_exp(vf>=vx)=0;
    t_dt_exp(vf>=vx)=0;
    %merging
    ai=a_ds_exp+a_dt_exp;
    ti=t_ds_exp+t_dt_exp;
    ti(ai>=amax | ai<=-amax)=[];
    ai(ai>=amax | ai<=-amax)=[];

    poss_num = sum((ai<amax) & (ai>-amax))
    %ploting
    k_plot = 50;
    kti= 0:1: k_plot-1;
    t_plot = kti'*ti/(k_plot-1);
    t_plot = t_plot';
    a_ki = repmat ( ai' , [1, k_plot]);

    vi_mat= repmat(vi_t, [length(vf), k_plot]);
    si= si_t + a_ki.*t_plot.^2./2 + vi_mat.*t_plot;
    t_plot = t_plot + ti_t;

    sk = si_t +  ai.*ti.^2/2 + vi_t.*ti;

%     txt1 = '$$\Delta s_{exp}$$';
%     text(-0.13*dt_exp,ds_exp,txt1,'Interpreter','latex','FontSize',12)
% 
%     txt1 = '$$\Delta t_{exp}$$';
%     text(dt_exp*0.95,-0.08*ds_exp,txt1,'Interpreter','latex','FontSize',12 )
    

    %% parent node
    plot ( ti_t, si_t, 'o', 'LineWidth',2 )
    txt1 = '$$n$$';
    text(ti_t-0.1,si_t-1,txt1,'Interpreter','latex','FontSize',12 )
    % t index
    txt1 = '$$n.t_k$$';
    text(origin(1),-1,txt1,'Interpreter','latex','FontSize',12 ,'HorizontalAlignment' , 'center')    
    % s index
    txt1 = '$$n.s_k$$';
    text(-0.03,origin(2),txt1,'Interpreter','latex','FontSize',12,'HorizontalAlignment' , 'right' ,'VerticalAlignment' , 'middle')
    %s reminder
    arrow3([origin(1)+dt_grid origin(2) 0 ], [origin(1)+dt_grid si_t 0], 'k-',0.5,1)
    arrow3([origin(1)+dt_grid si_t 0], [origin(1)+dt_grid origin(2) 0 ], 'k-',0.5,1)  
    txt1 = '$$n.s_r$$';
    text(origin(1)+dt_grid+0.02 , (origin(2)+si_t)/2, txt1,'Interpreter','latex','FontSize',12, 'VerticalAlignment' , 'middle', 'HorizontalAlignment' , 'left')
    
    %t reminder        
    arrow3([ti_t origin(2) 0 ], [origin(1) origin(2) 0], 'k-',0.5,1)
    arrow3([origin(1) origin(2) 0], [ti_t origin(2) 0 ], 'k-',0.5,1)
    txt1 = '$$n.t_r$$';
    text((origin(1)+ti_t)/2 , origin(2) ,txt1,'Interpreter','latex','FontSize',12 , 'VerticalAlignment' , 'bottom', 'HorizontalAlignment' , 'center' )
    
    %% transitions
    plot ( t_plot', si', 'LineWidth',2 )
    hold on
    %expand frame
    plot( [ti_t ti_t ti_t+dt_exp ti_t+dt_exp ti_t], [si_t si_t+ds_exp si_t+ds_exp si_t si_t], 'k-.')
    
    %% child nodes

    plot(ti_t+ti, sk, 'o', 'LineWidth',2);
    
    for cnt_j = 1: length(sk)
        
        txt1 = strcat('$$n''_',  strcat(num2str(cnt_j) ,'$$'));
        text(ti_t+ti(cnt_j)+0.01 ,sk(cnt_j)+0.05,txt1,'Interpreter','latex','FontSize',12 ,'HorizontalAlignment' , 'left','VerticalAlignment' , 'bottom')
    end
    
    arrow3([ti_t+dt_exp+0.1 sk(1) 0 ], [ti_t+dt_exp+0.1 2*ds_grid+origin(2) 0], 'k-',0.5,1)
    arrow3( [ti_t+dt_exp+0.1 2*ds_grid+origin(2) 0], [ti_t+dt_exp+0.1 sk(1) 0 ],'k-',0.5,1)
    txt1 = '$$n''_1.s_r$$';
    text(ti_t+dt_exp+0.12 , (sk(1)+2*ds_grid+origin(2))/2 ,txt1,'Interpreter','latex','FontSize',12, 'VerticalAlignment' , 'middle', 'HorizontalAlignment' , 'left' )
    
    
    arrow3([4*dt_grid+origin(1) 2*ds_grid+origin(2) 0 ], [ti_t+dt_exp 2*ds_grid+origin(2) 0], 'k-',0.5,1)
    arrow3( [ti_t+dt_exp 2*ds_grid+origin(2) 0],[4*dt_grid+origin(1) 2*ds_grid+origin(2) 0 ], 'k-',0.5,1)    
    txt1 = '$$n''_1.t_r$$';
    text((4*dt_grid+origin(1)+ti_t+dt_exp)/2 , 2*ds_grid+origin(2),txt1,'Interpreter','latex','FontSize',12, 'VerticalAlignment' , 'bottom', 'HorizontalAlignment' , 'center' )
    
    
    txt1 = '$$n''_1.t_k$$';
    text(4*dt_grid+origin(1),-1,txt1,'Interpreter','latex','FontSize',12 ,'HorizontalAlignment' , 'center')    
    txt1 = '$$n''_1.s_k$$';
    text(-0.03,2*ds_grid+origin(2),txt1,'Interpreter','latex','FontSize',12,'HorizontalAlignment' , 'right' ,'VerticalAlignment' , 'middle' )
    
    %% expand horizon arrows    
    arrow3([ti_t si_t+ds_exp+2 0 ], [ti_t+dt_exp si_t+ds_exp+2 0], 'k-',0.5,1)
    arrow3([ti_t+dt_exp si_t+ds_exp+2 0], [ti_t si_t+ds_exp+2 0 ], 'k-',0.5,1)
    txt1 = '$$\Delta t_{exp}$$';
    text(ti_t+0.5*dt_exp,si_t+ds_exp+2,txt1,'Interpreter','latex','FontSize',12 ,'HorizontalAlignment' , 'center','VerticalAlignment' , 'bottom')
    
    arrow3([ti_t-0.1 si_t 0 ], [ti_t-0.1 si_t+ds_exp 0], 'k-',0.5,1)
    arrow3([ti_t-0.1 si_t+ds_exp 0], [ti_t-0.1 si_t 0 ], 'k-',0.5,1)
    txt1 = '$$\Delta s_{exp}$$';
    text(ti_t-0.12,si_t+0.5*ds_exp,txt1,'Interpreter','latex','FontSize',12,'HorizontalAlignment' , 'right', 'VerticalAlignment' , 'middle')
    
    

end

