function plot_robot_velocities(t, v, font_size, line_thickness)
  num_velocities = size(v, 2);

  for i=1:num_velocities
    subplot(num_velocities, 1, i)
    plot(t, v(:,i), 'linewidth', line_thickness)
    hold on
    if i == 1
      title("Robot wheels velocities $[V_{m1} V_{m2} V_{m3}]^T$ over time")
    end
    grid on
    xlabel('Tempo (s)');
    ylabel(['$V_{m' num2str(i) '}(t)$ [m/s]']);
  end
end
