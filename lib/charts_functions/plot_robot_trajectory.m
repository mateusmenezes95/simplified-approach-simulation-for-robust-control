function plot_robot_trajectory(x, y, legend_name, line_spec, line_thickness)
  plot(x, y, line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
  legend
  grid on
  xlabel('x [m]')
  ylabel('y [m]')
end
