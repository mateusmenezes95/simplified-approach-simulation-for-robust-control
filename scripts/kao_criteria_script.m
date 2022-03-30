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
    preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);


q = 0.1:0.1:10;
Nmax_vector = zeros(size(q));
Cnorm_vector = zeros(size(q));
r=1000;
loop_index= 1;
for q_sample = q
    [Nmax, Cnorm] = kao_criteria(A_pred, B_pred, C_pred, q_sample, r, prediction_horizon, control_horizon, robot_discrete_model);
    Nmax_vector(loop_index) = Nmax;
    Cnorm_vector(loop_index) = Cnorm;
    loop_index = loop_index+1;
end

plot(q,Cnorm_vector)
xlabel('q')
ylabel('Norm_\infty')
figure
stem(q,Nmax_vector)
xlabel('q')
ylabel('Nmax')