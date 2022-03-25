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

function waypoints = generate_square_trajectory(square_size, nav_vel, sampling_period)
  path_nav_time = (square_size*4)/nav_vel;
  waypoints_qty = round(path_nav_time/sampling_period);
  waypoint_step = square_size/waypoints_qty;

  x1 = linspace(0, square_size, waypoints_qty);
  theta = [zeros(1,length(x1))'];
  y1 = linspace(waypoint_step, square_size, waypoints_qty-1);
  theta = [theta; deg2rad(ones(1,length(y1))*90)'];
  x2 = linspace(square_size-waypoint_step, 0, waypoints_qty-1);
  theta = [theta; deg2rad(ones(1,length(x2))*180)'];
  y2 = linspace(square_size-waypoint_step, waypoint_step, waypoints_qty-2);
  theta = [theta; deg2rad(ones(1,length(y2))*-135)'];

  x = [x1'; ones(1,length(y1))'; x2'; zeros(1, length(y2))'];
  y = [zeros(1, length(x1))'; y1'; ones(1, length(x2))'; y2'];

  waypoints = [x y theta]';
endfunction
