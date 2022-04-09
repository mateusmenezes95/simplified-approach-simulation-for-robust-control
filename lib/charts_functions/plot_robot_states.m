function plot_robot_states(t, x, states_name, font_size, line_thickness)
  num_states = size(x, 2);

  for i=1:3
    subplot(num_states, 1, i)
    plot(t, x(:,i), 'linewidth', line_thickness)
    hold on
    grid on
    xlabel('Tempo (s)');
    if i < 3
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    end
  end
end
