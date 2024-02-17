function body_fixed_vel_dot = nonlinear_map(body_fixed_vel, args)
	xk = body_fixed_vel;
	tau = args.tau;
	dynamic_model = args.dynamic_model;
	M = dynamic_model.rigid_body_inertia_matrix + dynamic_model.added_mass_system_inertia_matrix;
	C_RB = get_rigid_body_coriolis_and_centripetal_matrix(xk, dynamic_model.mass);
	C_A = get_added_mass_coriolis_and_centripetal_matrix(xk, dynamic_model.added_mass_system_inertia_matrix);
	C = C_RB + C_A;
	D_v = get_quadratic_damping_matrix(xk, dynamic_model.quadratic_damping_coefficients);
	D = dynamic_model.linear_damping_matrix + D_v;
	G = dynamic_model.gravity_vector;
	body_fixed_vel_dot = M\(tau - C*xk - D*xk - G);
end
