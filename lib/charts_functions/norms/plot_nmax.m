function plot_nmax(x_axis_ticks, x_axis_ticks_step, nmax_vec, title_str)
    stem(x_axis_ticks, nmax_vec, "Marker", "o", "Color", "r")
    title(title_str)
    grid on
    xlabel('q')
    ylabel('N_{max}')
    xlim([x_axis_ticks(2) max(x_axis_ticks)])
    maximum_nmax = max([nmax_vec(2:end)]); 
    yticks(0:1:maximum_nmax)
    xticks(x_axis_ticks(2):x_axis_ticks_step:max(x_axis_ticks))
    if max(nmax_vec) == 0
        ylim([0 1])
        yticks([0 1])
    else
        ylim([0 maximum_nmax])
    end
end
