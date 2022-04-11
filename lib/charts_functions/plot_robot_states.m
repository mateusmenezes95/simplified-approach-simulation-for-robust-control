function plot_robot_states(t, x, legend_name, line_spec, line_thickness)
  num_states = size(x, 2);
  states_name = {'v', 'v_n', '\omega'};

  for i=1:3
    subplot(num_states, 1, i)
    if(legend_name ~= -1)
      plot(t, x(:,i), line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
      legend
    else
      plot(t, x(:,i), line_spec, 'linewidth', line_thickness)
    end
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
