function plot_per_dof_values(t, args, desired_values, actual_values)
	dofs = size(desired_values, 2);
	if nargin == 3
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
	end

	if nargin > 3
		for i = 1:dofs
			subplot(dofs, 1, i)
			plot(t, actual_values(:, i), 'b-', 'LineWidth', 1.5)
			hold on
			plot(t, desired_values(:, i), '--r', 'LineWidth', 1.5)
			hold off
			legend('Atual', 'Desejado', 'Location', 'best')
			ylabel(args.y_labels{i}, 'Interpreter', 'latex')
			y_min = min(desired_values(:, i)) - args.y_min_offset;
			y_max = max(desired_values(:, i)) + args.y_max_offset;

			xlim([min(t) max(t)])
			ylim([y_min y_max])
			grid on
			ax = gca;
			outerpos = ax.OuterPosition;
			ti = ax.TightInset; 
			left = outerpos(1) + ti(1);
			bottom = outerpos(2) + ti(2);
			ax_width = outerpos(3) - ti(1) - ti(3) - 0.01;
			ax_height = outerpos(4) - ti(2) - ti(4);
			ax.Position = [left bottom ax_width ax_height];
		end
		xlabel('Time [s]')
	end
end
