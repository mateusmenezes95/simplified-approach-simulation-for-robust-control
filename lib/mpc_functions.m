% Function to define preditor params for free and forced response within
% prediction (Np) and control horizons (control_horizon). This is considered for discrete-time
% systems only.
function [A_pred, B_pred, C_pred] = preditor_params(A, B, C, Np, Nu)
  A_pred = [];
  C_pred = [];
  B_pred = [];
  B_pred_column = [];

  for i=1:Np
    % Computing A_pred
    A_pred= [A_pred;A^i];

    % Computing C_pred
    C_pred_column = [zeros(size(C,1)*(i-1),size(C,2)) ; C ; zeros(size(C,1)*(Np-i),size(C,2))];
    C_pred = [C_pred C_pred_column];

    % Computing B_pred_column;
    B_pred_column = [B_pred_column ; A^(i-1)*B];
  end

  for j=1:Nu
    B_pred_column_temp = [zeros(size(B,1)*(j-1),size(B,2)) ; B_pred_column];
    B_pred_column_temp(size(B_pred_column,1)+1:size(B_pred_column_temp,1), :) = [];
    B_pred = [B_pred B_pred_column_temp];
  end
end

function [Kw, Kmpc, Qaug, Raug] = get_mpc_gains(Ap, Bp, Cp, q, r, Np, Nu)
  Qaug = eye(size(Ap,2)*Np/2)*q;
  Raug = eye(size(Bp,2))*r;
  common_factor = inv((Bp'*Cp'*Qaug*Cp*Bp)+Raug)*Bp'*Cp'*Qaug';
  Kw = common_factor;
  Kmpc = common_factor*Cp*Ap;
end

% Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
function horizon_refs = get_horizon_references(k, prediction_horizon, future_refs)
  horizon_refs = [];
  for j=k:k+prediction_horizon-1
    if j > length(future_refs)
      horizon_refs = [horizon_refs; future_refs(:,end)];  % Keeps the last reference
    else
      horizon_refs = [horizon_refs; future_refs(:,j)];
    end
  end
end

% Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
function horizon_refs = get_square_trajectory_horizon_references(k, actual_pose, prediction_horizon, waypoints, nav_vel)
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
