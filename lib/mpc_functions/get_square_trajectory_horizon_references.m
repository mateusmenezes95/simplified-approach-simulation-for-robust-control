function horizon_refs = get_square_trajectory_horizon_references(waypoints, nav_vel, k, Np, pose)
    % Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
    horizon_refs = zeros(Np*3,1);

    k=k+1;

    x=pose(1);
    y=pose(2);
    theta=pose(3);
    tol = 0.01;

    if (k > length(waypoints))
        return
    end

    i=1;
    for j=k:k+Np-1
        Rz = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];

        if j > length(waypoints)
            x_ref = waypoints(1,end);
            y_ref = waypoints(2,end);
            theta_ref = waypoints(3,end);
        else
            x_ref = waypoints(1,j);
            y_ref = waypoints(2,j);
            theta_ref = waypoints(3,j);
        end

        phi = atan2(y_ref-y, x_ref-x);  % atan2(yr(k + j|k) - yr(k), xr(k + j|k) − xr(k))
        temp_vec = [nav_vel*cos(phi) nav_vel*sin(phi) theta_ref-theta]';
        temp_vec = Rz*temp_vec;
        horizon_refs(i:i+2,1) = temp_vec;
        i=i+3;
    end
end
