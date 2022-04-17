function plot_control_signals(t, u, legend_name, line_spec, line_thickness)
  control_signals = size(u, 2);

  for i=1:control_signals
    subplot(control_signals, 1, i)
    if(legend_name ~= -1)
      plot(t, u(:,i), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
      legend(Location="best")
    else
      plot(t, u(:,i), line_spec, 'linewidth', line_thickness)
    end
    hold on
    grid on
    xlabel('Tempo (s)');
    ylabel(['u_' num2str(i) ' [V]']);
  end
end
