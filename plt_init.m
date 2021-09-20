function [PAR, INPUT] = plt_init(PAR, INPUT)

%     figure('units','normalized','outerposition',[0 0 1 1])
%     hold on
    xlabel('$$x$$', 'Interpreter', 'Latex')
    ylabel('$$y$$', 'Interpreter', 'Latex')
    set(gca,'TickLabelInterpreter','latex')
    plot(INPUT.ROAD.X, INPUT.ROAD.Y, 'k--');
    hold on
    [x_inner, y_inner, x_outer, y_outer, R, unv, concavity, overlap]=parallel_curve(INPUT.ROAD.X, INPUT.ROAD.Y, INPUT.ROAD.width/2, 0, 0);
    
    plot(x_inner, y_inner, 'k-', 'LineWidth', 2);
    plot(x_outer, y_outer, 'k-', 'LineWidth', 2);
    xmin = min(min(x_outer),min(x_inner))-5;
    xmax = max(max(x_outer),max(x_inner))+5;
    ymin = min(min(y_outer),min(y_inner))-10;
    ymax = max(max(y_outer),max(y_inner))+10;
    xlim([xmin xmax]);
    ylim([ymin ymax]);
%     %compile
%     pbaspect([xmax-xmin ymax-ymin 1])
    drawnow