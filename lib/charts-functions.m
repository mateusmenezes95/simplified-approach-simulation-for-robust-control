% =============================================================================
% Defaul parameters
% =============================================================================

global font_size = 10;
global line_thickness = 1;
global y_axis_limits_offset = 0.2;

% =============================================================================
% Functions
% =============================================================================

function [y_min, y_max] = get_y_axis_limits(signal)
  global y_axis_limits_offset;
  y_min = min(signal) - y_axis_limits_offset;
  y_max = max(signal) + y_axis_limits_offset;
end

function plot_robot_states(t, x, states_name)
  global font_size;
  global line_thickness;

  num_states = size(x, 1);

  for i=1:num_states
    subplot(num_states, 1, i)
    plot(t, x(i,:), 'linewidth', line_thickness)
    grid on
    xlabel('Tempo (s)');
    if i < 3
      ylabel([states_name{i} ' [m/s]']);
    else
      ylabel([states_name{i} ' [rad/s]']);
    endif
  endfor
endfunction

function plot_robot_velocities(t, v)
  global font_size;
  global line_thickness;

  num_velocities = size(v, 1);

  for i=1:num_velocities
    subplot(num_velocities, 1, i)
    plot(t, v(i,:), 'linewidth', line_thickness)
    grid on
    xlabel('Tempo (s)');
    ylabel(['$V_{m' num2str(i) '}(t)$ [m/s]']);
  endfor
endfunction

function plot_robot_trajectory(x, y)
  global font_size;
  global line_thickness;

  plot(x, y, 'linewidth', line_thickness)
  grid on
  xlabel('x [m]')
  ylabel('y [m]')
endfunction