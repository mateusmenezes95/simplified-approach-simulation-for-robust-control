function plot_robot_velocities(t, v, legend_name, line_thickness)
    num_velocities = size(v, 2);

    for i=1:num_velocities
        subplot(num_velocities, 1, i)
        if(legend_name ~= -1)
            plot(t, v(:,i), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
            legend
        else
            plot(t, v(:,i), line_spec, 'linewidth', line_thickness)
        end
        hold on
        grid on
        xlabel('Tempo (s)');
        ylabel(['$V_{m' num2str(i) '}$ [m/s]']);
    end
end
