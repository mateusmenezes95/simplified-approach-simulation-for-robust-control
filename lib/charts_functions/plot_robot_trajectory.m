function plot_robot_trajectory(x, y, legend_name, line_spec, line_thickness)
  plot(x, y, line_spec, 'linewidth', line_thickness, 'DisplayName', legend_name)
  lh = legend;
  lh.Position(1) = 0.5 - lh.Position(3)/2; 
  lh.Position(2) = 0.5 - lh.Position(4)/2;
  grid on
  xlabel('x [m]')
  ylabel('y [m]')
end
