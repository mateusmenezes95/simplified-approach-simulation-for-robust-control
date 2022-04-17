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
%==============================================================================

% Simulated scenarios in the next foo loop:
% 1 - Reference tracking satisfatory without delay on the system. Thus a
% high q and low r
% 2 - System instability due the addition of a system delay keeping the
% same q and r of first scenario

r = 1;
q = 100;
Nmax_vec = [0 1];

line_style = '-b';

for i=1:length(Nmax_vec)
    %==============================================================================
    % MPC Initialization
    %==============================================================================
    Nmax = Nmax_vec(i);

    loop_step_params_str = ['q = ' num2str(q) ', r = ' num2str(r) ' and Nmax = ' num2str(Nmax)'];

    [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
    [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
    %==============================================================================

    %==============================================================================
    % Run Simulation
    %==============================================================================

    print_section_description(['Running Robot Simulation on Simulink with parameters: ' loop_step_params_str])
    sim_out = sim('./simulink/robot.slx');
    print_section_description("Robot Simulation Finished!")
    %==============================================================================

    %==============================================================================
    % Plot trajectories
    %==============================================================================
    figure('Name', ['Trajectory | params: ' loop_step_params_str])
    plot_robot_trajectory(x_trajectory, y_trajectory, 'referencia', '--k', line_thickness)
    xlim(trajectory_limits);
    ylim(trajectory_limits);
    hold on
    plot_robot_trajectory(sim_out.x, sim_out.y, 'Robô', line_style, line_thickness)
    %==============================================================================

    %==============================================================================
    % Plot robot poses
    %==============================================================================
    figure('Name',['Pose | params: ' loop_step_params_str])
    robot_pose = [sim_out.x, sim_out.y sim_out.theta];
    plot_robot_pose(sim_out.sim_time_sampled, robot_pose, -1, line_style, line_thickness)
    %==============================================================================

    %==============================================================================
    % Plot robot states
    %==============================================================================
    figure('Name',['States | params: ' loop_step_params_str])
    robot_states = [sim_out.v sim_out.vn sim_out.w];
    plot_robot_states(sim_out.sim_time_sampled, robot_states, -1, line_style, line_thickness)
    %==============================================================================

    %==============================================================================
    % Plot control signals
    %==============================================================================
    figure('Name',['Control Signals | params: ' loop_step_params_str])
    control_signals = [sim_out.u1 sim_out.u2 sim_out.u3];
    plot_control_signals(sim_out.sim_time_sampled, control_signals, -1, line_style, line_thickness)
    %==============================================================================

    %==============================================================================
    % Plot step responses
    %==============================================================================
    if plot_step_response
        figure('Name',['Step Response | params: ' loop_step_params_str])
        if i == 1
            state_step = [sim_out.step_v sim_out.step_vn sim_out.step_w];
            plot_robot_states(sim_out.sim_time_sampled, state_step, 'referencia', '--k', line_thickness)
        end
        state_step_response = [sim_out.step_reponse_v sim_out.step_reponse_vn sim_out.step_reponse_w];
        plot_robot_states(sim_out.sim_time_sampled, state_step_response, 'resposta', line_style, line_thickness)
        
        figure('Name',['Control Signals in Step Response | params: ' loop_step_params_str])
        u_step_response = [sim_out.step_reponse_u1 sim_out.step_reponse_u2 sim_out.step_reponse_u2];
        plot_control_signals(sim_out.sim_time_sampled, u_step_response, -1, line_style, line_thickness)
    end
end

% Simulated scenarios in the next for loop:
% MPC Tunning: R = 10000 and Q = 501
% The behavior of the system when the Kao criteria is ensured: Nmax < 2
% When the criteria is not Nmax = 3

r = 10000;
q = 501;
Nmax_vec = [0 2 3];

loop_step_params_str = ['q = ' num2str(q) ', r = ' num2str(r) ' and Nmax = ' num2str(Nmax_vec)];
print_section_description(['Running Robot Simulation on Simulink with parameters: ' loop_step_params_str])

plot_line_styles = ["-.m", ":b", "-r"];
figures = [];

for i=1:length(Nmax_vec)
    figure_idx = 1;

    Nmax = Nmax_vec(i);

    current_nmax_str = ['Nmax = ' num2str(Nmax)];

    [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
    [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);
    %==============================================================================

    %==============================================================================
    % Run Simulation
    %==============================================================================

    print_section_description(['Running Robot Simulation on Simulink to Nmax = ' num2str(Nmax)])
    sim_out = sim('./simulink/robot.slx');
    print_section_description("Robot Simulation Finished!")
    %==============================================================================

    %==============================================================================
    % Plot trajectories
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Trajectory | params: ' loop_step_params_str], i, figure_idx);
    if i == 1
        plot_robot_trajectory(x_trajectory, y_trajectory, 'referencia', '--k', line_thickness)
        xlim(trajectory_limits);
        ylim(trajectory_limits);
        hold on
    end
    plot_robot_trajectory(sim_out.x, sim_out.y, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot robot poses
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Pose | params: ' loop_step_params_str], i, figure_idx);
    robot_pose = [sim_out.x, sim_out.y sim_out.theta];
    plot_robot_pose(sim_out.sim_time_sampled, robot_pose, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot robot states
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['States | params: ' loop_step_params_str], i, figure_idx);
    robot_states = [sim_out.v sim_out.vn sim_out.w];
    plot_robot_states(sim_out.sim_time_sampled, robot_states, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot control signals
    %==============================================================================
    [figures, figure_idx] = select_figure(figures, ['Control Signals | params: ' loop_step_params_str], i, figure_idx);
    control_signals = [sim_out.u1 sim_out.u2 sim_out.u3];
    plot_control_signals(sim_out.sim_time_sampled, control_signals, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
    hold on
    %==============================================================================

    %==============================================================================
    % Plot step responses
    %==============================================================================
    if plot_step_response
        [figures, figure_idx] = select_figure(figures, ['Step Response | params: ' loop_step_params_str], i, figure_idx);
        if i == 1
            state_step = [sim_out.step_v sim_out.step_vn sim_out.step_w];
            plot_robot_states(sim_out.sim_time_sampled, state_step, 'referencia', '--k', line_thickness)
        end
        state_step_response = [sim_out.step_reponse_v sim_out.step_reponse_vn sim_out.step_reponse_w];
        plot_robot_states(sim_out.sim_time_sampled, state_step_response, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
        
        [figures, figure_idx] = select_figure(figures, ['Control Signals in Step Response | params: ' loop_step_params_str], i, figure_idx);
        u_step_response = [sim_out.step_reponse_u1 sim_out.step_reponse_u2 sim_out.step_reponse_u2];
        plot_control_signals(sim_out.sim_time_sampled, u_step_response, ['Nmax = ' num2str(Nmax)], plot_line_styles(i), line_thickness)
    end
end
