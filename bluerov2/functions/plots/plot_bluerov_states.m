function plot_bluerov_states(t, x, reference, line_spec, line_thickness)
  states_name = {'u', 'v', 'w', 'r'};
  num_states = size(states_name, 2);
	limit_offset = 0.05;

  for i=1:num_states
    subplot(num_states, 1, i)

    plot(t, reference(i,:), 'k-', 'linewidth', line_thickness, 'DisplayName', 'Reference')
    hold on
    plot(t, x(i,:), line_spec, 'linewidth', line_thickness, 'DisplayName', ['velocity ' states_name{i}])
    hold off
    legend('show', 'location', 'northeast')

    grid on
    xlabel('Time [s]');
    if i < num_states
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end

    min_value = min(min(x(i,:), reference(i,:)));
    max_value = max(max(x(i,:), reference(i,:)));

		ylim([(min_value - limit_offset) (max_value + limit_offset)])

		if i == 1
			title('Body-fixed velocities (robot states)')
		end

  end
end
