function plot_norms(x_axis_ticks, x_axis_ticks_step, norms_vec, title_str, ylabel_str)
    title(title_str)
    stem(x_axis_ticks, norms_vec, "Marker", ".")
    grid on
    xlabel('q')
    ylabel(ylabel_str)
    xlim([x_axis_ticks(2) max(x_axis_ticks)])
    xticks(x_axis_ticks(2):x_axis_ticks_step:max(x_axis_ticks))
end
