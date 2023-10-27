clear all, close all, clc
addpath(genpath("./")) % Add lib path to Octave script file search paths

% Comment the line below if you are using MATLAB
% pkg load control

run simulation_parameters

mass = 11.0;

inertia_moment_about_z_axix = 0.16;

added_mass.x_dot_u = -5.5;
added_mass.y_dot_v = -12.7;
added_mass.z_dow_w = -14.57;
added_mass.n_dot_r = -0.12;

damping.x_u = -25.15;
damping.y_v = -7.364;
damping.z_w = -17.955;
damping.m_q = -3.744;

rigit_body_inertia_matrix = ...
[
mass 0 0 0;
0 mass 0 0;
0 0 mass 0;
0 0 0 inertia_moment_about_z_axix;
];

added_mass_inertia_matrix = -diag([added_mass.x_dot_u added_mass.y_dot_v added_mass.z_dow_w added_mass.n_dot_r]);
linear_damping_matrix = -diag([damping.x_u damping.y_v damping.z_w damping.m_q]);

system_inertia_matrix = rigit_body_inertia_matrix + added_mass_inertia_matrix;

states = {'u', 'v', 'w', 'r'};
state_vector_size = 4;

A = -system_inertia_matrix\linear_damping_matrix;
B = inv(system_inertia_matrix);
C = eye(state_vector_size);

robot_continous_model = ss(A, B, C, 0, ...
                          'StateName', states, 'OutputName', states, ...
                          'Name', 'Robot Model');

% robot_discrete_model = c2d(robot_continous_model, sampling_period);
% 
% Ad = robot_discrete_model.A;
% Bd = robot_discrete_model.B;
% Cd = robot_discrete_model.C;
% 
% Aaug = ...
% [
%   Ad    zeros(size(Ad));
%   Cd*Ad eye(size(Cd,1),size(Ad,2))
% ];
% 
% Baug = ...
% [
%   Bd;
%   Cd*Bd
% ];
% 
% Caug = [zeros(state_vector_size) eye(state_vector_size)];
% 
% print_section_description("State Space Model Loaded")
