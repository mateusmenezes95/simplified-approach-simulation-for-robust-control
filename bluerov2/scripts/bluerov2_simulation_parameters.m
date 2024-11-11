addpath(genpath("../lib"))

sampling_period = 10e-3;
prediction_horizon = 30;  % Prediction Horizon
control_horizon = 20;  % Control Horizon

base_path_for_fig_save = "/home/mateus/ufba_ws/pgcomp-ufba-latex/figuras";

print_section_description("Simulation parameters loaded")
