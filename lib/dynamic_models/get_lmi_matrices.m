function [Almi, Blmi, Clmi, Dlmi] = get_lmi_matrices(model_params, dt, np, nu, r_weight, q_weight) 
    % Augmented state-space due the integrator addition
    [Aaug, Baug, Caug, A, B, C, D] = get_model_matrices(model_params, dt);
  
    amount_of_states = size(A, 2);
    amount_of_inputs = size(B, 2);
  
    % Matrices to estimate the future states and control signals
    [Acal, Bcal, Ccal] = preditor_params(Aaug, Baug, Caug, np, nu);
    [Kw, Kmpc, Q, R] = get_mpc_gains(Acal, Bcal, Ccal, q_weight, r_weight, np, nu);
    Kmpc = Kmpc(1:amount_of_inputs, :); % Receding horizon control
  
    z = tf('z', dt);
    C_z = -Kmpc*[eye(amount_of_states);(z/(z-1))*C];
    Cz_ss = ss(C_z);
  
    Ac = Cz_ss.A;
    Bc = Cz_ss.B;
    Cc = Cz_ss.C;
    Dc = Cz_ss.D;
  
    Almi = [A+B*Dc*C B*Cc zeros(3); Bc*C Ac zeros(3); Dc*C Cc zeros(3)];
    Blmi = [B; zeros(3); zeros(3)];
    Clmi = [Dc Cc -eye(3)];
    Dlmi = zeros(3);
end
