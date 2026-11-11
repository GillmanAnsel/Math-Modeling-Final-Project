% Strategy 3: Two-Pursuer Pure Pursuit
% Both pursuers continuously steer toward the evader's current position.
clear; clc; close all;

%% Model parameters
v_p = 60 / 3.6;
v_e = 50 / 3.6;
R_p = 1.5;
R_e = 0.5;
w_p_max = v_p / R_p;
w_e_max = v_e / R_e;
l_cap = 1.0;
r_jink = 2.5;
T_max = 15.0;
dt = 0.005;
start_distances = [15, 20, 25, 30, 35];
flank_angles = deg2rad([15, 20, 25, 30, 35]);
plot_folder = fullfile(pwd, 'strategy_3_plots');
if ~exist(plot_folder, 'dir')
    mkdir(plot_folder);
end

%% Scenario trajectories and separation plots
for scenario = 1:numel(start_distances)
    [t, x_E, y_E, theta_E, x_P1, y_P1, theta_P1, ...
        x_P2, y_P2, theta_P2, d1, d2, end_idx] = run_simulation( ...
        start_distances(scenario), flank_angles(scenario), r_jink, ...
        deg2rad(110), v_p, v_e, R_p, R_e, l_cap, T_max, dt);

    figure('Color', 'w', 'Position', [80, 80, 1050, 700]);
    hold on; grid on; axis equal;
    plot(x_E(1:end_idx), y_E(1:end_idx), 'b-', 'LineWidth', 2.2, ...
        'DisplayName', 'Evader (E)');
    plot(x_P1(1:end_idx), y_P1(1:end_idx), 'r--', 'LineWidth', 2.0, ...
        'DisplayName', 'Pursuer 1 (P_1 - Pure Pursuit)');
    plot(x_P2(1:end_idx), y_P2(1:end_idx), 'g-.', 'LineWidth', 2.0, ...
        'DisplayName', 'Pursuer 2 (P_2 - Pure Pursuit)');
    plot(x_E(1), y_E(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
        'HandleVisibility', 'off');
    plot(x_P1(1), y_P1(1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
        'HandleVisibility', 'off');
    plot(x_P2(1), y_P2(1), 'gd', 'MarkerSize', 8, 'MarkerFaceColor', 'g', ...
        'HandleVisibility', 'off');
    plot(x_E(end_idx), y_E(end_idx), 'ko', 'MarkerSize', 12, ...
        'MarkerFaceColor', 'y', 'DisplayName', 'Capture Point');
    th = linspace(0, 2*pi, 100);
    plot(x_E(end_idx) + l_cap*cos(th), y_E(end_idx) + l_cap*sin(th), ...
        'm--', 'LineWidth', 2.2, 'DisplayName', 'Capture Radius (1.0 m)');
    arrow_end = max(1, end_idx - round(0.5 / dt));
    if arrow_end > 1
        idx = round(linspace(1, arrow_end, 5));
        quiver(x_E(idx), y_E(idx), cos(theta_E(idx)), sin(theta_E(idx)), ...
            0.35, 'b', 'HandleVisibility', 'off');
        quiver(x_P1(idx), y_P1(idx), cos(theta_P1(idx)), sin(theta_P1(idx)), ...
            0.35, 'r', 'HandleVisibility', 'off');
        quiver(x_P2(idx), y_P2(idx), cos(theta_P2(idx)), sin(theta_P2(idx)), ...
            0.35, 'g', 'HandleVisibility', 'off');
    end
    xlabel('Horizontal Position x (m)');
    ylabel('Vertical Position y (m)');
    title(sprintf('Strategy 3: Two-Pursuer Pure Pursuit (r_{det} = %.0f m, t_{cap} = %.2f s)', ...
        start_distances(scenario), t(end_idx)));
    legend('Location', 'best');
    set(gca, 'FontSize', 12, 'Box', 'on');
    exportgraphics(gcf, fullfile(plot_folder, ...
        sprintf('scenario_%d_trajectory.png', scenario)), 'Resolution', 200);

    figure('Color', 'w', 'Position', [180, 120, 850, 550]);
    hold on; grid on;
    plot(t(1:end_idx), d1(1:end_idx), 'r-', 'LineWidth', 1.8, ...
        'DisplayName', 'd_1(t) (Pursuer 1)');
    plot(t(1:end_idx), d2(1:end_idx), 'g-', 'LineWidth', 1.8, ...
        'DisplayName', 'd_2(t) (Pursuer 2)');
    yline(l_cap, 'k--', 'Capture Threshold (l = 1.0 m)', 'LineWidth', 1.5);
    yline(r_jink, 'b:', sprintf('Jink Trigger (r_{jink} = %.1f m)', r_jink), ...
        'LineWidth', 1.5);
    xlabel('Time t (s)');
    ylabel('Distance to Prey (m)');
    title('Strategy 3: Pair Separation Distances vs. Time');
    legend('Location', 'southoutside', 'NumColumns', 2);
    set(gca, 'FontSize', 12, 'Box', 'on');
    exportgraphics(gcf, fullfile(plot_folder, ...
        sprintf('scenario_%d_separation.png', scenario)), 'Resolution', 200);
    close all;
end

%% Jink-distance sweep
jink_distances = 1.0:0.25:8.0;
capture_times = zeros(numel(start_distances), numel(jink_distances));
for scenario = 1:numel(start_distances)
    for j = 1:numel(jink_distances)
        capture_times(scenario, j) = capture_time(start_distances(scenario), ...
            flank_angles(scenario), jink_distances(j), deg2rad(110), ...
            v_p, v_e, R_p, R_e, l_cap, T_max, dt);
    end
end
plot_sweep(jink_distances, capture_times, start_distances, ...
    'Jink distance (m)', 'Strategy 3: Capture Time for Jink Distances', ...
    fullfile(plot_folder, 'jink_distance_capture_time.png'));

%% Jink-angle sweep
angle_offsets = deg2rad(0:5:180);
angle_capture_times = zeros(numel(start_distances), numel(angle_offsets));
for scenario = 1:numel(start_distances)
    for j = 1:numel(angle_offsets)
        angle_capture_times(scenario, j) = capture_time(start_distances(scenario), ...
            flank_angles(scenario), r_jink, angle_offsets(j), ...
            v_p, v_e, R_p, R_e, l_cap, T_max, dt);
    end
end
plot_sweep(rad2deg(angle_offsets), angle_capture_times, start_distances, ...
    'Jink angle offset (degrees)', ...
    sprintf('Strategy 3: Capture Time by Jink Angle (r_{jink} = %.1f m)', r_jink), ...
    fullfile(plot_folder, 'jink_angle_capture_time.png'));
close all;

function [t, x_E, y_E, theta_E, x_P1, y_P1, theta_P1, ...
    x_P2, y_P2, theta_P2, d1, d2, end_idx] = run_simulation( ...
    r_det, alpha, r_jink, jink_angle, v_p, v_e, R_p, R_e, l_cap, T_max, dt)
    N = round(T_max / dt);
    w_p_max = v_p / R_p;
    w_e_max = v_e / R_e;
    t = (0:N-1) * dt;
    x_E = zeros(1, N); y_E = zeros(1, N); theta_E = zeros(1, N);
    x_P1 = zeros(1, N); y_P1 = zeros(1, N); theta_P1 = zeros(1, N);
    x_P2 = zeros(1, N); y_P2 = zeros(1, N); theta_P2 = zeros(1, N);
    d1 = zeros(1, N); d2 = zeros(1, N);
    x_P1(1) = -r_det * cos(alpha); y_P1(1) = r_det * sin(alpha);
    x_P2(1) = -r_det * cos(alpha); y_P2(1) = -r_det * sin(alpha);
    is_jinking = false;
    theta_target = 0;
    end_idx = N;

    for k = 1:N-1
        d1(k) = hypot(x_E(k) - x_P1(k), y_E(k) - y_P1(k));
        d2(k) = hypot(x_E(k) - x_P2(k), y_E(k) - y_P2(k));
        if min(d1(k), d2(k)) <= l_cap
            end_idx = k;
            break;
        end

        theta_LOS1 = atan2(y_E(k) - y_P1(k), x_E(k) - x_P1(k));
        theta_LOS2 = atan2(y_E(k) - y_P2(k), x_E(k) - x_P2(k));
        w_P1 = bounded_turn(theta_LOS1, theta_P1(k), w_p_max, dt);
        w_P2 = bounded_turn(theta_LOS2, theta_P2(k), w_p_max, dt);

        if is_jinking && min(d1(k), d2(k)) > r_jink + 1.5
            is_jinking = false;
        end
        if ~is_jinking && min(d1(k), d2(k)) <= r_jink
            is_jinking = true;
            if d1(k) <= d2(k)
                theta_target = theta_LOS1 + jink_angle;
            else
                theta_target = theta_LOS2 + jink_angle;
            end
        end
        if is_jinking
            w_E = bounded_turn(theta_target, theta_E(k), w_e_max, dt);
        else
            nearest_los = theta_LOS1;
            if d2(k) < d1(k)
                nearest_los = theta_LOS2;
            end
            w_E = bounded_turn(nearest_los, theta_E(k), w_e_max, dt);
        end

        x_P1(k+1) = x_P1(k) + v_p*cos(theta_P1(k))*dt;
        y_P1(k+1) = y_P1(k) + v_p*sin(theta_P1(k))*dt;
        theta_P1(k+1) = theta_P1(k) + w_P1*dt;
        x_P2(k+1) = x_P2(k) + v_p*cos(theta_P2(k))*dt;
        y_P2(k+1) = y_P2(k) + v_p*sin(theta_P2(k))*dt;
        theta_P2(k+1) = theta_P2(k) + w_P2*dt;
        x_E(k+1) = x_E(k) + v_e*cos(theta_E(k))*dt;
        y_E(k+1) = y_E(k) + v_e*sin(theta_E(k))*dt;
        theta_E(k+1) = theta_E(k) + w_E*dt;
    end
    d1(end_idx) = hypot(x_E(end_idx) - x_P1(end_idx), y_E(end_idx) - y_P1(end_idx));
    d2(end_idx) = hypot(x_E(end_idx) - x_P2(end_idx), y_E(end_idx) - y_P2(end_idx));
end

function w = bounded_turn(target, heading, max_rate, dt)
    delta = atan2(sin(target - heading), cos(target - heading));
    w = sign(delta) * min(abs(delta) / dt, max_rate);
end

function result = capture_time(r_det, alpha, r_jink, jink_angle, ...
    v_p, v_e, R_p, R_e, l_cap, T_max, dt)
    [t, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, end_idx] = ...
        run_simulation(r_det, alpha, r_jink, jink_angle, ...
        v_p, v_e, R_p, R_e, l_cap, T_max, dt);
    result = t(end_idx);
end

function plot_sweep(x_values, values, start_distances, x_label, title_text, output_file)
    figure('Color', 'w', 'Position', [80, 80, 1200, 850]);
    for scenario = 1:numel(start_distances)
        subplot(3, 2, scenario);
        bar(x_values, values(scenario, :), 1.0, ...
            'FaceColor', [0.25, 0.50, 0.85], 'EdgeColor', 'none');
        hold on; grid on;
        [max_time, max_idx] = max(values(scenario, :));
        plot(x_values(max_idx), max_time, 'r^', ...
            'MarkerSize', 9, 'MarkerFaceColor', 'r');
        xline(x_values(max_idx), 'r--', 'LineWidth', 1.2);
        xlabel(x_label);
        ylabel('Capture time (s)');
        title(sprintf('Scenario %d (r_{det} = %.0f m)', ...
            scenario, start_distances(scenario)));
        legend('Capture time', sprintf('Maximum: %.2f', x_values(max_idx)), ...
            'Location', 'best');
        set(gca, 'FontSize', 10, 'Box', 'on');
    end
    sgtitle(title_text);
    exportgraphics(gcf, output_file, 'Resolution', 200);
end
