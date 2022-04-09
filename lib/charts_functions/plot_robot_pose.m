function plot_robot_pose(t, p, line_thickness)
    labels = {'x [m]', 'y [m]', '\theta [rad]'};
    for i=1:3
        subplot(3, 1, i)
        plot(t, p(:,i), 'LineWidth', line_thickness)
        hold on
        grid on
        ylabel(labels{i});
        if i == 3
            xlabel('Tempo (s)');
        end
    end
end
