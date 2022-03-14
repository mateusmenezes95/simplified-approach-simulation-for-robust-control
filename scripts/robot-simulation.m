addpath(genpath("../lib")) % Add lib path to Octave script file search paths

run util-functions
run robot-model
run charts-functions
run trajectory-functions

sim_time = 5;
step_begin = 1;
step_begin_idx = floor(step_begin/dt);

t = 0:dt:sim_time;

u = zeros(size(discrete_model.A, 1), size(t, 2));
u(:,step_begin_idx:end) = ones(size(discrete_model.A, 1),
                              (size(t, 2)-step_begin_idx+1));

[x_trajectory, y_trajectory, v_world_ref] = generate_circle_trajectory(1, 1, t);

xt(:,1) = [0 0 0]';                      % x = [v vn w]'
vt(:,1) = inverse_kinematics(xt(:,1));    % v = [vm1 vm2 vm3]'
pt(:,1) = [0 0 0]';                      % p = [xr yr theta]'

K = eye(3)*0.2;

u(:,1) = [0 0 0]';

k=1;
for i=1:length(t)-1;
  ut(:,i) = u(:,k);
  [xt(:,i+1), vt(:,i+1)] = robot(xt(:,i), ut(:,i));
  pt(:,i+1) = compute_odometry(pt(:,i), xt(:,i));

  theta = pt(3,i);
  xreft(:,i) = rotz(theta)*v_world_ref(:,i);

  if mod(i,100) == 0
    k = k+1;
    x(:,k) = xt(:,i);
    xref(:,k) = xreft(:,i);
    u(:,k) = K*(x(:,k)-xref(:,k));
  endif

endfor

xr = pt(1,:);
yr = pt(2,:);

figure(1)
plot_robot_states(t,xt,discrete_model.stname);
figure(2)
plot_robot_velocities(t,vt);
figure(3)
plot_robot_trajectory(xr, yr)
