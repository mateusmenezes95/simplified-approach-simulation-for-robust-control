function plot_bluerov_states(t, x, legend_name, line_spec, line_thickness)
  states_name = {'u', 'v', 'w', 'r'};
  num_states = size(states_name, 2);

  for i=1:num_states
    subplot(num_states, 1, i)
    if(legend_name ~= -1)
      plot(t, x(i,:), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
      legend
    else
      plot(t, x(i,:), line_spec, 'linewidth', line_thickness)
    end
    hold on
    grid on
    xlabel('Time [s]');
    if i < num_states
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end

		if i == 1
			title('Body-fixed velocities (robot states)')
		end

  end
end
