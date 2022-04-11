function plot_robot_pose(t, p, legend_name, line_spec, line_thickness)
    labels = {'x [m]', 'y [m]', '\theta [rad]'};
    for i=1:3
        subplot(3, 1, i)
        if(legend_name ~= -1)
            plot(t, p(:,i), line_spec, 'LineWidth', line_thickness, 'DisplayName', legend_name)
            legend
        else
            plot(t, p(:,i), line_spec, 'LineWidth', line_thickness)
        end
        hold on
        grid on
        ylabel(labels{i});
        if i == 3
            xlabel('Tempo (s)');
        end
    end
end
