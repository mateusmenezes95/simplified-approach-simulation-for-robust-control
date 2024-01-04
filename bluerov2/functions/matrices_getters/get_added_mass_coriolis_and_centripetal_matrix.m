function C_A = get_added_mass_coriolis_and_centripetal_matrix(xk, added_mass_matrix)
	% get_added_mass_coriolis_and_centripetal_matrix - Returns the added mass coriolis and centripetal matrix for a given
	% body-fixed velocity vector and a diagonal added mass matrix.
  u = xk(1);
  v = xk(2);
  x_dot_u = added_mass_matrix(1,1);
  y_dot_v = added_mass_matrix(2,2);
  C_A = [0,          0,         0, y_dot_v*v;
         0,          0,         0, -x_dot_u*u;
         0,          0,         0,   0;
         -y_dot_v*v, x_dot_u*u, 0,   0];
end
