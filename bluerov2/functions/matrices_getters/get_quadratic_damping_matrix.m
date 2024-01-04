function D_v = get_quadratic_damping_matrix(xk, coefficient)
	% get_quadratic_damping_matrix - Returns the quadratic damping matrix D(v) from the body-fixed velocity xk and the
	% quadratic damping coefficients.
  u = xk(1);
  v = xk(2);
  w = xk(3);
  r = xk(4);
  D_v = -diag([coefficient.x_abs_u_u*abs(u), ...
              coefficient.y_abs_v_v*abs(v), ...
              coefficient.z_abs_w_w*abs(w), ...
              coefficient.n_abs_r_r*abs(r)]);
end
