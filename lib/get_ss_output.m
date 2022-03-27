function [xkplus1, yk] = get_ss_output(xk, ss_matrix, uk)
% This is the Euler Forward Algorithm implementation. Therefore, the matrices A
% here must be informat to A = (I + dt*A) and B = dt*B, where dt is the
% integration step size. This assumption is due the the state space approximation
% x(t+dt) - x(t)
% -------------- = Ax(t) + Bu(t)
%       dt
    xkplus1 = (ss_matrix.a*xk) + (ss_matrix.b*uk);
    yk = ss_matrix.c*xk + (ss_matrix.d*uk);
end