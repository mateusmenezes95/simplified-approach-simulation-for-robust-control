clear all, close all, clc
addpath(genpath("./")) % Add lib path to Octave script file search paths

% Comment the line below if you are using MATLAB
% pkg load control

run simulation_parameters

Bv = 0.7;      % viscous friction relative to v (N/m/s)
Bvn = 0.7;     % viscous friction relative to v n (N/m/s)
Bw = 0.011;   % viscous friction relative to ω (N/rad/s)

Cv = 0.28;    % Coulomb friction relative to v (N )
Cvn = 0.14;    % Coulomb friction relative to v n (N )
Cw = 0.0086;  % Coulomb friction relative to ω (N.m)

M = 1.551;   % robots mass (kg)
Mt = 0.0062;  % robots inertial momentum (kg.m 2 )

L = 0.1;     % robots radius (m)
wr = 0.0505;  % wheels radius (m)

nr = 19/1;    % motors gear’s reduction rate
La = 0.00011; % armature inductance (H)
Ra = 1.69;    % armature resistance (Ω)
Kv = 0.0059;  % motor velocity constant (V olts/rad/s)
Kt = 0.0059;  % torque constant (N.m/A)

gmma = deg2rad(30); % Angle between orthoganal axes and robot wheels (rad)

% Forward Kinematic transformation matrix
global G

G = [
  0           -1        L;
  cos(gmma)   sin(gmma) L;
  -cos(gmma)  sin(gmma) L
];

% Plant dynamic discrete matrix

a11 = (-(3*(nr^2)*Kt*Kv)/(2*M*Ra*(wr^2)))-(Bv/M);
a22 = a11;
a33 = (-(3*(L^2)*(nr^2)*Kt*Kv)/(Mt*Ra*(wr^2)))-(Bw/Mt);

A = ...
[
  a11  0    0;
  0    a22  0;
  0    0    a33;
];

B = ...
[
  0     sqrt(3)/(2*M)   -sqrt(3)/(2*M)
  -1/M  1/(2*M)         1/(2*M)
  L/Mt  L/Mt           L/Mt
];
B = ((nr*Kt)/(Ra*wr))*B;

C = eye(3);
Ce = C;

states = {'v', 'vn', 'w'};
state_vector_size = 3;

robot_continous_model = ss(A, B, C, 0, ...
                          'StateName', states, 'OutputName', states, ...
                          'Name', 'Robot Model');

robot_discrete_model = c2d(robot_continous_model, sampling_period);

Ad = robot_discrete_model.A;
Bd = robot_discrete_model.B;
Cd = robot_discrete_model.C;

Aaug = ...
[
  Ad    zeros(size(Ad));
  Cd*Ad eye(size(Cd,1),size(Ad,2))
];

Baug = ...
[
  Bd;
  Cd*Bd
];

Caug = [zeros(state_vector_size) eye(state_vector_size)];

print_section_description("State Space Model Loaded")

function v = inverse_kinematics(x, G)
  v = G*x;
end

function x = forward_kinematics(v, G)
  x = G\v;
end
