function horizon_refs = get_square_trajectory_horizon_references(k, actual_pose, prediction_horizon, waypoints, nav_vel)
% Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
  x=1;
  y=2;
  theta=3;
  horizon_refs = [];
  for j=k:k+prediction_horizon-1
    if j > length(waypoints)
      phi = atan2(waypoints(y,end)-actual_pose(y), waypoints(x,end)-actual_pose(x))  % atan2(yr(k + j|k) - yr(k), xr(k + j|k) − xr(k))
      temp_vec = [nav_vel*cos(phi) nav_vel*sin(phi) (waypoints(theta,end) - actual_pose(theta))]';
      temp_vec = rotz(actual_pose(theta))*temp_vec;
      horizon_refs = [horizon_refs; temp_vec];
    else
      phi = atan2(waypoints(y,j)-actual_pose(y), waypoints(x,j)-actual_pose(x))  % atan2(yr(k + j|k) - yr(k), xr(k + j|k) − xr(k))
      temp_vec = [nav_vel*cos(phi) nav_vel*sin(phi) (waypoints(theta,j) - actual_pose(theta))]';
      temp_vec = rotz(actual_pose(theta))*temp_vec
      horizon_refs = [horizon_refs; temp_vec];
    end
  end
end
