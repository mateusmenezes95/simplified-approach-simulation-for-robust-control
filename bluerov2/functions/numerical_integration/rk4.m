% Given y' = f(x) and y(x0) = y0, this function returns y(x)
function yn_plus_1 = rk4(yn, xn, time_step, derivative_func, derivative_func_args)
	h = time_step;
  k1 = feval(derivative_func, xn, derivative_func_args);
  k2 = feval(derivative_func, xn + h/2*k1, derivative_func_args);
  k3 = feval(derivative_func, xn + h/2*k2, derivative_func_args);
  k4 = feval(derivative_func, xn + h*k3, derivative_func_args);
  yn_plus_1 = yn + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
