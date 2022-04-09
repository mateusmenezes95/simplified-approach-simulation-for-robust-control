function [u_min, u_max] = get_axis_limits(signal, offset)
  u_min = min(signal) - offset;
  u_max = max(signal) + offset;
end
