function plot_robot_trajectory(x, y, font_size, line_thickness, line_style)
  plot(x, y, 'linewidth', line_thickness, 'LineStyle', line_style)
  grid on
  xlabel('x [m]')
  ylabel('y [m]')
end
