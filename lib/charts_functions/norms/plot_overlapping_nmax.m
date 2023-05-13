function plot_overlapping_nmax(x_axis_ticks, x_axis_ticks_step, ...
                               underlay_nmax, overlay_nmax, legends)
    stem(x_axis_ticks, underlay_nmax, "Marker", "x", "Color", "r")
    hold on
    stem(x_axis_ticks, overlay_nmax, "Marker", ".", "Color", "b")
    legend(legends{1}, legends{2});
    grid on
    xlabel('q')
    ylabel('N_{max}')
    xlim([x_axis_ticks(2) max(x_axis_ticks)])
    maximum_nmax = max([overlay_nmax(2:end); underlay_nmax(2:end)], [], 'all'); 
    yticks(0:1:maximum_nmax)
    xticks(x_axis_ticks(2):x_axis_ticks_step:max(x_axis_ticks))
    if max(overlay_nmax) == 0
        ylim([0 1])
        yticks([0 1])
    else
        ylim([0 maximum_nmax])
    end
end
