function [fig_vec_updated, next_idx] = select_figure(fig_vec, fig_name, loop_step, fig_idx)
%SELECT_FIGURE Summary of this function goes here
%   Select figure according to loop step
    if loop_step == 1
        fig_id = figure('Name', fig_name);
        fig_vec(fig_idx) = fig_id;
    else
        figure(fig_vec(fig_idx))
    end
    next_idx = fig_idx + 1;
    fig_vec_updated = fig_vec;
end
