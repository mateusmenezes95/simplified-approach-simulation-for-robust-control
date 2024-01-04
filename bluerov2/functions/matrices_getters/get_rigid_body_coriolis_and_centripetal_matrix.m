function C_RB = get_rigid_body_coriolis_and_centripetal_matrix(xk, mass)
	% get_rigid_body_coriolis_and_centripetal_matrix Creates the rigid body Coriolis matrix from the velocity vector in the body-fixed frame
	%
	% C_A = get_rigid_body_coriolis_matrix(xk, added_mass_matrix) where xk is the velocity vector in the body-fixed frame
	% and mass is the mass of the rigid body
	u = xk(1);
	v = xk(2);
	C_RB = [0,      0,       0, -mass*v;
					0,      0,       0, mass*u;
					0,      0,       0,   0;
					mass*v, -mass*u, 0,   0];
end
