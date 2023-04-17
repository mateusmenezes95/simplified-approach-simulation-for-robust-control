clc
clear all

addpath(genpath("../lib")) 
addpath(genpath("../lib/mpc_functions")) 

run simulation_parameters

nominal_mass = 1.551;
nominal_inertia = 0.0062;

[Aaug, Baug, Caug, Ad, Bd, Cd] = get_model_augmented_matrices(nominal_mass, nominal_inertia, sampling_period);

r = 1;
q = 100;

[Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, prediction_horizon, control_horizon);
[Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q, r, prediction_horizon, control_horizon);

Kmpc = Kmpc(1:size(Bd, 2), :); % Receding horizon control

no=size(Cd,1);
ni=size(Bd,2);
ns=size(Ad,2);

Almi = Aaug-Baug*Kmpc;
Blmi = Baug;
Clmi = -Kmpc;
Dlmi = zeros(3);

% Calcula norma inf (LMI)
n=size(Almi,1); % dimensão do sistema aumentado

% Define variáveis para LMI ( comandos definidos pelo Yalmip )
Pd=sdpvar(n);
Gd=sdpvar(n,n,'full');
mu=sdpvar(1);

%Define a lista de LMIs
ineqs=[];

ineqs=[ineqs,[Pd Almi*Gd Blmi zeros(n,ni);...
           Gd'*Almi' Gd+Gd'-Pd zeros(n,ni) Gd'*Clmi'; ...
            Blmi' zeros(ni,n) eye(ni) Dlmi';...
             zeros(ni,n) Clmi*Gd Dlmi eye(ni)*mu]>=0];

% Define objetivo para otimização 
objective = mu;

% Exectua as LMIs
opts = sdpsettings;
opts.savesolveroutput =1 ;
opts.solver = 'sdpt3';

yalmipdiagnostics = solvesdp(ineqs,objective,opts);

% Recupera as variáveis
mu = double(mu);

fprintf('--------------')
fprintf('Norma via função de transferência - alternativa')
sys1=ss(Almi,Blmi,Clmi,Dlmi,-1);
norm(sys1,inf)

fprintf('Norma via LMI - Lema 2 / Teorema 4')
sqrt(mu)

function [Aaug, Baug, Caug, Ad, Bd, Cd] = get_model_augmented_matrices(mass, inertia, sampling_period)
  Bv = 0.7;      % viscous friction relative to v (N/m/s)
  Bvn = 0.7;     % viscous friction relative to v n (N/m/s)
  Bw = 0.011;   % viscous friction relative to ω (N/rad/s)

  %M = 1.551;   % robots mass (kg)
  M = mass;
  %J = 0.0062;  % robots inertial momentum (kg.m 2 )
  J = inertia;  % robots inertial momentum (kg.m 2 )

  L = 0.1;     % robots radius (m)
  wr = 0.0505;  % wheels radius (m)

  nr = 19/1;    % motors gear’s reduction rate
  Ra = 1.69;    % armature resistance (Ω)
  Kv = 0.0059;  % motor velocity constant (V olts/rad/s)
  Kt = 0.0059;  % torque constant (N.m/A)

  % Plant dynamic continuous matrix
  a11 = (-(3*(nr^2)*Kt*Kv)/(2*M*Ra*(wr^2)))-(Bv/M);
  a22 = a11;
  a33 = (-(3*(L^2)*(nr^2)*Kt*Kv)/(J*Ra*(wr^2)))-(Bw/J);

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
    L/J  L/J           L/J
  ];
  B = ((nr*Kt)/(Ra*wr))*B;

  C = eye(3);

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
end
