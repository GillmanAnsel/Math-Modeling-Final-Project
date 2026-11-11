function create_trajectory_animation_3(t, x_E, y_E, theta_E, ...
    x_P1, y_P1, theta_P1, x_P2, y_P2, theta_P2, l_cap, output_file)
    fig = figure('Color', 'w', 'Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    all_x = [x_E, x_P1, x_P2];
    all_y = [y_E, y_P1, y_P2];
    pad = max(4, 0.08 * max(max(all_x) - min(all_x), max(all_y) - min(all_y)));
    xlim(ax, [min(all_x) - pad, max(all_x) + pad]);
    ylim(ax, [min(all_y) - pad, max(all_y) + pad]);
    xlabel(ax, 'Horizontal Position x (m)');
    ylabel(ax, 'Vertical Position y (m)');
    h_E = plot(ax, NaN, NaN, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Evader');
    h_P1 = plot(ax, NaN, NaN, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Pursuer 1');
    h_P2 = plot(ax, NaN, NaN, 'g-.', 'LineWidth', 2.0, 'DisplayName', 'Pursuer 2');
    h_arrow_E = quiver(ax, NaN, NaN, NaN, NaN, 1.5, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 2);
    h_arrow_P1 = quiver(ax, NaN, NaN, NaN, NaN, 1.5, 'r', 'LineWidth', 1.8, 'MaxHeadSize', 2);
    h_arrow_P2 = quiver(ax, NaN, NaN, NaN, NaN, 1.5, 'g', 'LineWidth', 1.8, 'MaxHeadSize', 2);
    legend(ax, 'Location', 'best');
    title_handle = title(ax, '');
    video = VideoWriter(output_file, 'MPEG-4');
    video.FrameRate = 30;
    open(video);
    frame_step = max(1, round(0.02 / (t(2) - t(1))));
    for k = 1:frame_step:numel(t)
        set(h_E, 'XData', x_E(1:k), 'YData', y_E(1:k));
        set(h_P1, 'XData', x_P1(1:k), 'YData', y_P1(1:k));
        set(h_P2, 'XData', x_P2(1:k), 'YData', y_P2(1:k));
        set(h_arrow_E, 'XData', x_E(k), 'YData', y_E(k), ...
            'UData', cos(theta_E(k)), 'VData', sin(theta_E(k)));
        set(h_arrow_P1, 'XData', x_P1(k), 'YData', y_P1(k), ...
            'UData', cos(theta_P1(k)), 'VData', sin(theta_P1(k)));
        set(h_arrow_P2, 'XData', x_P2(k), 'YData', y_P2(k), ...
            'UData', cos(theta_P2(k)), 'VData', sin(theta_P2(k)));
        set(title_handle, 'String', sprintf('Strategy 3 Trajectory (t = %.2f s)', t(k)));
        drawnow;
        writeVideo(video, getframe(fig));
    end
    close(video);
    close(fig);
end
