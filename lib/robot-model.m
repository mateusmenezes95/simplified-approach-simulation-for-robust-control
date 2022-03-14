clear all, close all, clc
addpath(genpath("./")) % Add lib path to Octave script file search paths

pkg load control

run util-functions

print_section_description("Plant State Space Model")

Bv = 0.7;      # viscous friction relative to v (N/m/s)
Bvn = 0.7;     # viscous friction relative to v n (N/m/s)
Bw = 0.011;   # viscous friction relative to ω (N/rad/s)
Cv = 0.28;    # Coulomb friction relative to v (N )
Cvn = 0.14;    # Coulomb friction relative to v n (N )
Cw = 0.0086;  # Coulomb friction relative to ω (N.m)
M = 1.551;   # robots mass (kg)
Mt = 0.0062;  # robots inertial momentum (kg.m 2 )
L = 0.1;     # robots radius (m)
r = 0.0505;  # wheels radius (m)
nr = 19/1;    # motors gear’s reduction rate
La = 0.00011; # armature inductance (H)
Ra = 1.69;    # armature resistance (Ω)
Kv = 0.0059;  # motor velocity constant (V olts/rad/s)
Kt = 0.0059;  # torque constant (N.m/A)
gmma = deg2rad(30); # Angle between orthoganal axes and robot wheels (rad)

# Forward Kinematic transformation matrix
global G = ...
[
  0           -1        L;
  cos(gmma)   sin(gmma) L;
  -cos(gmma)  sin(gmma) L
];


sampling_period = 60e-3;
global dt = sampling_period/100;

# Plant dynamic discrete matrix

a11 = a22 = (-(3*(nr^2)*Kt*Kv)/(2*M*Ra*(r^2)))-(Bv/M);
a33 = (-(3*(L^2)*(nr^2)*Kt*Kv)/(Mt*Ra*(r^2)))-(Bw/Mt);

A = \
[
  a11  0    0;
  0    a22  0;
  0    0    a33;
];
Ae = (eye(size(A)) + dt*A); # To use in Euler Approximation Rule

B = \
[
  0     sqrt(3)/(2*M)   -sqrt(3)/(2*M)
  -1/M  1/(2*M)         1/(2*M)
  L/Mt  L/Mt           L/Mt
];
B = ((nr*Kt)/(Ra*r))*B;
Be = dt*B;

C = eye(3);

states = {'v', 'vn', 'w'};
continous_model = ss(A, B, C, 0,
                    'stname', states, 'outname', states,
                    'name', 'Robot Model');

global euler_rule_model = ss(Ae, Be, C, 0,
                             'stname', states, 'outname', states,
                             'name', 'Robot Model');
discrete_model = c2d(continous_model, sampling_period)

function v = inverse_kinematics(x)
  global G;
  v = G*x;
endfunction

function x = forward_kinematics(v)
  global G;
  x = inv(G)*v;
endfunction

function [xt_plus_dt, v] = robot(x0, u)
  global euler_rule_model;
  [xt_plus_dt, yt] = get_ss_output(x0,euler_rule_model, u);
  v = inverse_kinematics(x0); % Return [vm1 vm2 vm3]'
endfunction

function p = compute_odometry(p0, x)
  global dt;
  p = rotz(p0(3,1))*x*dt + p0;
endfunction
