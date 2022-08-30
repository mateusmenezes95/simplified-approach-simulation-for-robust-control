addpath(genpath("./lib"))
addpath(genpath("./lib/mpc_functions"))
addpath(genpath("./lib/chart_functions"))

run simulation_parameters
run robot_model

robot_vel = 0.05;
square_trajectory = generate_square_trajectory(1, robot_vel, sampling_period);
x_trajectory = square_trajectory(1,:);
y_trajectory = square_trajectory(2,:);
waypoints_qty = length(square_trajectory);

sim_time = 100;  % Choosen due results inpection

%==============================================================================
% Plot Parameters
%==============================================================================
font_size = 10;
line_thickness = 1;
y_axis_limits_offset = 0.2;
figure_idx = 1;
plot_step_response = false;
trajectory_limits =  [-0.05 1.05];
clipped_trajectory_start_time = 39;
clipped_trajectory_end_time = 46;

line_style = '-b';

if version('-release') == "2022a"
    simulink_model = "robot_varying_delay.slx";
else
    return;
end

print_section_description(strcat("Using ", simulink_model, " simulink file"));

% Simulated scenarios in the next for loop:
% - MPC Tunning: R = 10000 and Q = 501
% - Maximum tolerated delay based on Kao criteria: N = 2
% - The behavior of the system:
% ---- When the Kao criteria is ensured: N <= 2
% ---- When the criteria is violated: N = 3

r = 10000;
q = 500;
nmin = 1;
nmax = 2;

varying_delay_str = ['N varying between ' num2str(nmin) ' and ' num2str(nmax)];

loop_step_params_str = ['q = ' num2str(q) ', r = ' num2str(r) ' and ' varying_delay_str];
print_section_description(['Running Robot Simulation on Simulink with parameters: ' loop_step_params_str])

plot_line_styles = "-b";
clipped_plot_line_styles = "-b";
figures = [];

figure_idx = 1;
i = 1;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
%==============================================================================

%==============================================================================
% Run Simulation
%==============================================================================

print_section_description(['Running Robot Simulation on Simulink to ' varying_delay_str])
sim_out = sim('./simulink/robot_varying_delay.slx');
print_section_description("Robot Simulation Finished!")
%==============================================================================

%==============================================================================
% Plot trajectories
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['Trajectory | params: ' loop_step_params_str], i, figure_idx);
plot_robot_trajectory(x_trajectory, y_trajectory, 'Referência', '--k', line_thickness)
xlim(trajectory_limits);
ylim(trajectory_limits);
hold on
plot_robot_trajectory(sim_out.x, sim_out.y, 'Robô', plot_line_styles, line_thickness)
%==============================================================================

%==============================================================================
% Plot clipped trajectories
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['Clipped Trajectory | params: ' loop_step_params_str], i, figure_idx);
f = figure(figures(figure_idx-1));
f.Position = [1435 712 732 617];
start_idx = find(sim_out.sim_time_sampled >= clipped_trajectory_start_time, 1);
end_idx = find(sim_out.sim_time_sampled >= clipped_trajectory_end_time, 1);
plot_robot_trajectory(sim_out.x(start_idx:end_idx), sim_out.y(start_idx:end_idx), '', clipped_plot_line_styles, line_thickness)
legend('hide')
%==============================================================================

%==============================================================================
% Plot robot poses
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['Pose | params: ' loop_step_params_str], i, figure_idx);
robot_pose = [sim_out.x, sim_out.y sim_out.theta];
plot_robot_pose(sim_out.sim_time_sampled, robot_pose, -1, plot_line_styles, line_thickness)
%==============================================================================

%==============================================================================
% Plot robot states
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['States | params: ' loop_step_params_str], i, figure_idx);
robot_states = [sim_out.v sim_out.vn sim_out.w];
plot_robot_states(sim_out.sim_time_sampled, robot_states, -1, plot_line_styles, line_thickness)
%==============================================================================

%==============================================================================
% Plot control signals
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['Control Signals | params: ' loop_step_params_str], i, figure_idx);
control_signals = [sim_out.u1 sim_out.u2 sim_out.u3];
plot_control_signals(sim_out.sim_time_sampled, control_signals, -1, plot_line_styles, line_thickness)
%==============================================================================

%==============================================================================
% Plot cliped control signals
%==============================================================================
[figures, figure_idx] = select_figure(figures, ['Clipped Control Signals | params: ' loop_step_params_str], i, figure_idx);
f = figure(figures(figure_idx-1));
control_signals = [sim_out.u1(start_idx:end_idx) sim_out.u2(start_idx:end_idx) sim_out.u3(start_idx:end_idx)];
plot_control_signals(sim_out.sim_time_sampled(start_idx:end_idx), control_signals, -1, clipped_plot_line_styles, 1.5*line_thickness)
for m=1:3
    subplot(3,1,m)
    xlim([clipped_trajectory_start_time clipped_trajectory_end_time])
end
%==============================================================================

figure()
%k = 0:length(sim_out.sim_time_sampled)-1; 
%stem(k, sim_out.nvar, "Marker",".")
stem(start_idx:end_idx, sim_out.nvar(start_idx:end_idx), "Marker",".")
ylim([0 nmax+0.2])
yticks(0:1:2)
xlabel('k')
ylabel('N')
xlim([start_idx end_idx])
%xlim([0 max(k)])
grid on

