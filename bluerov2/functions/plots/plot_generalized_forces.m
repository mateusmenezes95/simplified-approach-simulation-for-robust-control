function plot_generalized_forces (t, u, legend_name, line_spec, line_thickness, ylabel_prefix)
	generalized_forces_name = {'X', 'Y', 'Z', 'N'}; % According to SNAME notation
	control_signals = size(generalized_forces_name, 2);
	limit_offset = 0.5;

	for i=1:control_signals
		subplot(control_signals, 1, i)

		if(legend_name ~= -1)
			plot(t, u(i,:), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
			legend(Location="best")
		else
			plot(t, u(i,:), line_spec, 'linewidth', line_thickness)
		end
		hold on
		grid on
		xlabel('Time [s]');

    if i < control_signals
      ylabel([generalized_forces_name{i} ' [N]']);
    else
      ylabel([generalized_forces_name{i} ' [Nm]']);
    end

		ylim([(min(u(i,:)) - limit_offset) (max(u(i,:)) + limit_offset)])
		xlim([min(t) max(t)])

		ax = gca;
		outerpos = ax.OuterPosition;
		ti = ax.TightInset; 
		left = outerpos(1) + ti(1);
		bottom = outerpos(2) + ti(2);
		ax_width = outerpos(3) - ti(1) - ti(3) - 0.01;
		ax_height = outerpos(4) - ti(2) - ti(4);
		ax.Position = [left bottom ax_width ax_height];

		% if i == 1
		% 	title('Generalized forces (control signals)')
		% end
	end
end
