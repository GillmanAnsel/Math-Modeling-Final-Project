function create_trajectory_animation_1(t, x_E, y_E, theta_E, x_P, y_P, theta_P, ...
    l_cap, output_file)
    fig = figure('Color', 'w', 'Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    all_x = [x_E, x_P];
    all_y = [y_E, y_P];
    pad = max(4, 0.08 * max(max(all_x) - min(all_x), max(all_y) - min(all_y)));
    xlim(ax, [min(all_x) - pad, max(all_x) + pad]);
    ylim(ax, [min(all_y) - pad, max(all_y) + pad]);
    xlabel(ax, 'Horizontal Position x (m)');
    ylabel(ax, 'Vertical Position y (m)');
    h_E = plot(ax, NaN, NaN, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Evader');
    h_P = plot(ax, NaN, NaN, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Pursuer');
    h_arrow_E = quiver(ax, NaN, NaN, NaN, NaN, 1.5, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 2);
    h_arrow_P = quiver(ax, NaN, NaN, NaN, NaN, 1.5, 'r', 'LineWidth', 1.8, 'MaxHeadSize', 2);
    legend(ax, 'Location', 'best');
    title_handle = title(ax, '');
    video = VideoWriter(output_file, 'MPEG-4');
    video.FrameRate = 30;
    open(video);
    frame_step = max(1, round(0.02 / (t(2) - t(1))));
    for k = 1:frame_step:numel(t)
        set(h_E, 'XData', x_E(1:k), 'YData', y_E(1:k));
        set(h_P, 'XData', x_P(1:k), 'YData', y_P(1:k));
        set(h_arrow_E, 'XData', x_E(k), 'YData', y_E(k), ...
            'UData', cos(theta_E(k)), 'VData', sin(theta_E(k)));
        set(h_arrow_P, 'XData', x_P(k), 'YData', y_P(k), ...
            'UData', cos(theta_P(k)), 'VData', sin(theta_P(k)));
        set(title_handle, 'String', sprintf('Strategy 1 Trajectory (t = %.2f s)', t(k)));
        drawnow;
        writeVideo(video, getframe(fig));
    end
    close(video);
    close(fig);
end
