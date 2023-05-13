function plot_overlapping_norms(x_axis_ticks, x_axis_ticks_step, underlay_norms, ...
                                overlay_norms, legends)
    stem(x_axis_ticks, underlay_norms, "Marker", ".", "Color", "b")
    hold on
    stem(x_axis_ticks, overlay_norms, "Marker", "x", "Color", "r")
    legend(legends{1}, legends{2}, "Location", "northwest");
    grid on
    xlabel('q')
    ylabel('H_\infty')
    xlim([x_axis_ticks(2) max(x_axis_ticks)])
    xticks(x_axis_ticks(2):x_axis_ticks_step:max(x_axis_ticks))
end
