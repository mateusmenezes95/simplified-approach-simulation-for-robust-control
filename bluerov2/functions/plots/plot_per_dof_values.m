function plot_per_dof_values(t, args, desired_values, actual_values)
	dofs = size(desired_values, 2);
	for i = 1:dofs
		subplot(dofs, 1, i)
		plot(t, desired_values(:, i), 'r', 'LineWidth', 1.5)
		ylabel(args.y_labels{i}, 'Interpreter', 'latex')

		y_min = min(desired_values(:, i)) - args.y_min_offset;
		y_max = max(desired_values(:, i)) + args.y_max_offset;
		
		xlim([min(t) max(t)])
		ylim([y_min y_max])
		grid on
	end
	xlabel('Time [s]')

	if nargin > 3
		for i = 1:dofs
			subplot(dofs, 1, i)
			hold on
			plot(t, actual_values(:, i), 'b', 'LineWidth', 1.5)
			hold off
		end
		legend('Desejado', 'Atual')
	end
end
