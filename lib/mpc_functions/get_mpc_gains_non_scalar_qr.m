function [Kw, Kmpc, Qaug, Raug] = get_mpc_gains_non_scalar_qr(Ap, Bp, Cp, q, r, Np, Nu)
  Q = {};
  R = {};

  for i = 1:Np
    Q{i} = q;
  end

  for j = 1:Nu
    R{j} = r;
  end

  Qaug = blkdiag(Q{:});
  Raug = blkdiag(R{:});

  common_factor = ((Bp'*Cp'*Qaug*Cp*Bp)+Raug)\(Bp'*Cp'*Qaug');
  Kw = common_factor; % Needs review
  Kmpc = common_factor*Cp*Ap;
end
