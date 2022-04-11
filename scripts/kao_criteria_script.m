clc
clear all
close all

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions")) 

run simulation_parameters
run robot_model

print_section_description("Running script for Kao criteria...")

[A_pred, B_pred, C_pred] = ... 
    preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);

q = 1:10:1001;
Nmax_vector = zeros(size(q));
Cnorm_vector = zeros(size(q));

r=1;

loop_index= 1;
for q_sample = q
    [Nmax, Cnorm] = kao_criteria(A_pred, B_pred, C_pred, q_sample, r, ...
        prediction_horizon, control_horizon, robot_discrete_model);
    Nmax_vector(loop_index) = Nmax;
    Cnorm_vector(loop_index) = Cnorm;
    loop_index = loop_index + 1;
end

figure(1)
stem(q, Nmax_vector)
grid on
xlabel('q')
ylabel('N_{max}')
xlim([min(q) max(q)])
if max(Nmax_vector) == 0
    ylim([0 1])
    yticks([0 1])
else
    ylim([0 max(Nmax_vector)])
end

figure(2)
plot(q, Cnorm_vector)
grid on
xlabel('q')
ylabel('Norm_\infty')

print_section_description("Script for Kao criteria finished...")
