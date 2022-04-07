function [Kw, Kmpc, Qaug, Raug] = get_mpc_gains(Ap, Bp, Cp, q, r, Np, Nu)
  Qaug = eye(size(Ap,2)*Np/2)*q;
  Raug = eye(size(Bp,2))*r;
  common_factor = ((Bp'*Cp'*Qaug*Cp*Bp)+Raug)\(Bp'*Cp'*Qaug');
  Kw = common_factor; % Needs review
  Kmpc = common_factor*Cp*Ap;
end
