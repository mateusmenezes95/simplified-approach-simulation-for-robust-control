function plot_robot_trajectory(x, y, legend_name, line_spec, line_thickness)
  if (legend_name == -1)
    plot(x, y, line_spec, 'linewidth', line_thickness)
  else
    plot(x, y, line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
    lh = legend;
    lh.Position(1) = 0.5 - lh.Position(3)/2; 
    lh.Position(2) = 0.5 - lh.Position(4)/2;
  end
  grid on
  [x_min, x_max] = get_axis_limits(x, 0.1);
  [y_min, y_max] = get_axis_limits(y, 0.1);
  axis([x_min, x_max, y_min, y_max])
  xlabel('x [m]')
  ylabel('y [m]')
end
