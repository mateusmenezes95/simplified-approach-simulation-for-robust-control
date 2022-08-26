function [Nmax, CNorm] = kao_criteria(Apred, Bpred, Cpred, q, r, Np, Nu, model)
    %KAO_CRITERIA Summary of this function goes here
    %   Detailed explanation goes here
    [Kw, Kmpc, Qaug, Raug] = get_mpc_gains(Apred, Bpred, Cpred, q, r, ...
                                Np, Nu);
    state_size = size(model.A,1);
    Kmpc = Kmpc(1:state_size, :); % Receding horizon control
    
    z = tf('z', model.ts);
    
    A = model.A;
    B = model.B;
    C = model.C;
    D = model.D;
  
    C_z = Kmpc*[eye(state_size);(z/(z-1))*C];
    Cz_ss = ss(C_z);

    Ac = Cz_ss.A;
    Bc = Cz_ss.B;
    Cc = Cz_ss.C;
    Dc = Cz_ss.D;

    Aaug = [A-(B*Dc*C) B*Cc; -Bc*C Ac];
    Baug = [B;zeros(state_size)];
    Caug = [-Dc*C Cc];
    
    complementary_sensitivity = ss(Aaug, Baug, Caug, 0, model.ts);
    CNorm = norm(complementary_sensitivity*((z-1)/z), Inf);

    Nmax = 0;
    if CNorm < 1
        Nmax = floor(1/CNorm);
    end
end
