function plot_control_signals(t, u, font_size, line_thickness)
  control_signals = size(u, 2);

  for i=1:control_signals
    subplot(control_signals, 1, i)
    plot(t, u(:,i), 'linewidth', line_thickness)
    hold on
    grid on
    xlabel('Tempo (s)');
    ylabel(['$u_{m' num2str(i) '}(t)$ [V]']);
  end
end
