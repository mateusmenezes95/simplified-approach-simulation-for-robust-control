clc
clear all
close all

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions")) 

print_section_description("Starting script for Kao criteria")

run simulation_parameters
run robot_model

print_section_description("Calculating controller gains")

[A_pred, B_pred, C_pred] = ... 
    preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon)

[Kw, Kmpc, Qaug, Raug] = get_mpc_gains(A_pred, B_pred, C_pred, q, r, ...
                            prediction_horizon, control_horizon);

Kmpc = Kmpc(1:state_vector_size, :); % Receding horizon control

print_section_description("Estimating Nmax from Kao criteria")

z = tf('z', sampling_period);

P_z = tf(robot_discrete_model); % Only valid for C = eye(3)
C_z = Kmpc * [eye(3) ; (z/(z-1))*C];

C_sens = feedback(C_z*P_z, eye(state_vector_size));

Kao_sys = ((z-1)/z)*C_sens;

kao_Nmax = norm(Kao_sys, Inf)

