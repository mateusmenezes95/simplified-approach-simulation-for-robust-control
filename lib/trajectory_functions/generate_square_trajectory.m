function waypoints = generate_square_trajectory(square_size, nav_vel, sampling_period)
  path_nav_time = (square_size*4)/nav_vel;
  waypoints_qty = ceil((path_nav_time/sampling_period)/4);
  waypoint_step = square_size/waypoints_qty;

  x1 = ones(1,waypoints_qty);
  theta = zeros(1,length(x1))';
  y1 = ones(1,waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(y1))*90)'];
  x2 = zeros(1,waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(x2))*180)'];
  y2 = zeros(1,waypoints_qty);
  theta = [theta; deg2rad(ones(1,length(y2))*0)'];

  x = [x1'; ones(1,length(y1))'; x2'; zeros(1, length(y2))'];
  y = [zeros(1, length(x1))'; y1'; ones(1, length(x2))'; y2'];

  waypoints = [x y theta]';
end
