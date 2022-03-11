function print_section_description(description)
    disp("===================================================================")
    disp(description)
    disp("===================================================================")
endfunction

% This is the Euler Forward Algorithm implementation. Therefore, the matrices A
% here must be informat to A = (I - dt*A) and B = dt*B, where dt is the
% integration step size.
function [xkplus1, yk] = get_ss_output(xk, ss_matrix = ss(0,0,0,0), uk)
    xkplus1 = (ss_matrix.a*xk) + (ss_matrix.b*uk);
    yk = ss_matrix.c*xk + (ss_matrix.d*uk);
end
