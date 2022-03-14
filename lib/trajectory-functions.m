function [x_points, y_points, v] = generate_circle_trajectory(radius, v_nav, t)
  dx = radius/(length(t)/4);
  x_points = -radius:dx:radius;
  y_points = sqrt(1-(x_points.^2));
  x_points = [x_points flip(x_points(2:end))];
  y_points = [y_points -1*y_points(2:end)];
  initial_theta = deg2rad(90);  % The robot starts the trajectory to the left pointing forward

  for j=1:(length(x_points)-1)
    phi(j) = atan2((y_points(j+1)-y_points(j)),
                    (x_points(j+1)-x_points(j)));
    v(1,j) = v_nav*cos(phi(j));
    v(2,j) = v_nav*sin(phi(j));
    if j>1
      v(3,j) = phi(j) - phi(j-1);
    else
      v(3,j) = phi(j) - initial_theta;
    endif
  endfor
endfunction
