function [Nmax, CNorm] = kao_criteria(Apred, Bpred, Cpred, q, r, Np, Nu, model)
    %KAO_CRITERIA Summary of this function goes here
    %   Detailed explanation goes here
    [Kw, Kmpc, Qaug, Raug] = get_mpc_gains(Apred, Bpred, Cpred, q, r, ...
                                Np, Nu);
    state_size = size(model.A,1);
    Kmpc = Kmpc(1:state_size, :); % Receding horizon control
    
    z = tf('z', model.ts);
    
    P_z = tf(model); % Only valid for C = eye(3)
    
    C_z = Kmpc * [eye(3) ; (z/(z-1))*model.C];
    
    C_sens = feedback(series(C_z, P_z), eye(state_size), -1);
    
    Kao_sys = ((z-1)/z)*C_sens;

    CNorm = norm(Kao_sys, Inf);
    Nmax = -1;
    if CNorm < 1
        Nmax = floor(1/CNorm);
    end
end

