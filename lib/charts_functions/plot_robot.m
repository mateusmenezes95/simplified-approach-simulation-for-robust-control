function plot_robot(x, y)
%PLOT_ROBOT Plot robot heading in 2D chart
    dx = 0.025;
    dy = dx;
    line([x, x], [dy, -dy],'Color', 'r')
    line([x, x+dx], [dy, 0],'Color', 'r')
    line([x, x+dx], [-dy, 0],'Color', 'r')
end

