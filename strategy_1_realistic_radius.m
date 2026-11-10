% strategy_1_realistic_radius.m
% Part 1: Solitary Hunting with a realistic pursuer turning radius
clear; clc; close all;

%% 1. Model Parameters
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)
R_p = 5.0;               % Pursuer min turning radius (m)
R_e = 0.5;               % Evader min turning radius (m)

w_p_max = v_p / R_p;     % Max angular velocity of pursuer (3.33 rad/s)
w_e_max = v_e / R_e;     % Max angular velocity of evader (27.78 rad/s)

l_cap = 1.0;             % Capture radius (m)
r_det = 25.0;            % Initial detection distance (m) [15, 50]
r_jink = 3.0;            % Critical distance where prey triggers jink (m)
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.005;              % Time step (s)
N = round(T_max / dt);
start_distances = [15, 20, 25, 30, 35];
start_angles = deg2rad([0, 15, -15, 30, -30]);
plot_folder = fullfile(pwd, 'strategy_1_realistic_radius_plots');
if ~exist(plot_folder, 'dir')
    mkdir(plot_folder);
end

for scenario = 1:5
r_det = start_distances(scenario);
%% 2. Initial Conditions (t = 0)
alpha_1 = start_angles(scenario);

% Preallocate state arrays
x_E = zeros(1, N); y_E = zeros(1, N); theta_E = zeros(1, N);
x_P = zeros(1, N); y_P = zeros(1, N); theta_P = zeros(1, N);
dist_P = zeros(1, N);
t_vec = (0:N-1) * dt;

% Set t = 0 values
x_E(1) = 0; y_E(1) = 0; theta_E(1) = 0;
x_P(1) = -r_det * cos(alpha_1);
y_P(1) =  r_det * sin(alpha_1);
theta_P(1) = 0;

captured = false;
end_idx = N;
is_jinking = false;
theta_target = 0;

%% 3. Time-Stepping Simulation Loop (Forward Euler)
for k = 1:N-1
    dist_P(k) = hypot(x_E(k) - x_P(k), y_E(k) - y_P(k));
    
    if dist_P(k) <= l_cap
        captured = true;
        end_idx = k;
        break;
    end
    
    % --- Pursuer Control (Pure Pursuit) ---
    theta_LOS = atan2(y_E(k) - y_P(k), x_E(k) - x_P(k));
    dtheta_P = atan2(sin(theta_LOS - theta_P(k)), cos(theta_LOS - theta_P(k)));
    if abs(dtheta_P) <= w_p_max * dt
        w_P = dtheta_P / dt;
    else
        w_P = w_p_max * sign(dtheta_P);
    end
    
    % --- Evader Control (Break Turn with Overshoot Reset) ---
    if is_jinking && dist_P(k) > r_jink + 1.5
        is_jinking = false;
    end

    if ~is_jinking && dist_P(k) <= r_jink
        is_jinking = true;
        theta_target = theta_LOS + pi / 2;
    end

    if is_jinking
        dtheta_E = atan2(sin(theta_target - theta_E(k)), ...
            cos(theta_target - theta_E(k)));
        if abs(dtheta_E) <= w_e_max * dt
            w_E = dtheta_E / dt;
        else
            w_E = w_e_max * sign(dtheta_E);
        end
    else
        theta_flee = theta_LOS;
        dtheta_E = atan2(sin(theta_flee - theta_E(k)), cos(theta_flee - theta_E(k)));
        if abs(dtheta_E) <= w_e_max * dt
            w_E = dtheta_E / dt;
        else
            w_E = w_e_max * sign(dtheta_E);
        end
    end
    
    % --- Euler Kinematic Updates ---
    x_P(k+1) = x_P(k) + v_p * cos(theta_P(k)) * dt;
    y_P(k+1) = y_P(k) + v_p * sin(theta_P(k)) * dt;
    theta_P(k+1) = theta_P(k) + w_P * dt;
    
    x_E(k+1) = x_E(k) + v_e * cos(theta_E(k)) * dt;
    y_E(k+1) = y_E(k) + v_e * sin(theta_E(k)) * dt;
    theta_E(k+1) = theta_E(k) + w_E * dt;
end

dist_P(end_idx) = hypot(x_E(end_idx) - x_P(end_idx), y_E(end_idx) - y_P(end_idx));

%% 4. Figure 1: Combined 2D Trajectory Plot (All Players on Same Axis)
figure('Color', 'w', 'Position', [80, 80, 1050, 700]);
hold on; grid on; axis equal;

% Plot trajectories
plot(x_E(1:end_idx), y_E(1:end_idx), 'b-', 'LineWidth', 2.2, 'DisplayName', 'Evader Trajectory (E)');
plot(x_P(1:end_idx), y_P(1:end_idx), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Pursuer Trajectory (P_1)');

% Plot Start Positions (t = 0)
plot(x_E(1), y_E(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
plot(x_P(1), y_P(1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% Plot End Positions (t = T_final)
plot(x_E(end_idx), y_E(end_idx), 'ko', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'y', 'DisplayName', 'Capture Point');
plot(x_P(end_idx), y_P(end_idx), 'ro', 'MarkerSize', 6, ...
    'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% Plot Capture Radius Circle around Evader's Final Position
th_circ = linspace(0, 2*pi, 100);
plot(x_E(end_idx) + l_cap*cos(th_circ), y_E(end_idx) + l_cap*sin(th_circ), ...
    'm--', 'LineWidth', 2.2, 'DisplayName', 'Capture Radius (1.0 m)');

% Plot Heading Arrows along the Path
arrow_end = max(1, end_idx - round(0.5 / dt));
if arrow_end > 1
    idx_arrows = round(linspace(1, arrow_end, 6));
    quiver(x_E(idx_arrows), y_E(idx_arrows), cos(theta_E(idx_arrows)), sin(theta_E(idx_arrows)), ...
        0.35, 'b', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    quiver(x_P(idx_arrows), y_P(idx_arrows), cos(theta_P(idx_arrows)), sin(theta_P(idx_arrows)), ...
        0.35, 'r', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
xlabel('Horizontal Position x (m)', 'FontSize', 13);
ylabel('Vertical Position y (m)', 'FontSize', 13);
title(sprintf('Part 1: Solitary Pursuit Trajectories (r_{det} = %.0f m, Min Dist = %.2f m)', ...
    r_det, min(dist_P(1:end_idx))), 'FontSize', 14);
legend('Location', 'best');
exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_trajectory.png', scenario)), 'Resolution', 200);

%% 5. Figure 2: Separation Distance Plot
figure('Color', 'w', 'Position', [180, 120, 850, 550]);
hold on; grid on;
plot(t_vec(1:end_idx), dist_P(1:end_idx), 'k-', 'LineWidth', 2.0);
yline(l_cap, 'r--', 'Capture Threshold (l = 1.0 m)', 'LineWidth', 1.5);
yline(r_jink, 'b:', 'Jink Trigger (r_{jink} = 3.0 m)', 'LineWidth', 1.5);
set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
xlabel('Time t (s)', 'FontSize', 13);
ylabel('Separation Distance d_1(t) (m)', 'FontSize', 13);
title('Part 1: Predator-Prey Separation Distance vs. Time', 'FontSize', 14);
exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_separation.png', scenario)), 'Resolution', 200);
close all;
end

%% 6. Jink-Distance Sweep: Capture Time Histogram
jink_distances = 1.0:0.25:8.0;
capture_times = zeros(5, numel(jink_distances));

for scenario = 1:5
    for j = 1:numel(jink_distances)
        capture_times(scenario, j) = simulate_capture_time( ...
            start_distances(scenario), start_angles(scenario), ...
            jink_distances(j), v_p, v_e, R_p, R_e, l_cap, T_max, dt);
    end
end

figure('Color', 'w', 'Position', [80, 80, 1200, 850]);
for scenario = 1:5
    subplot(3, 2, scenario);
    bar(jink_distances, capture_times(scenario, :), 1.0, ...
        'FaceColor', [0.25, 0.50, 0.85], 'EdgeColor', 'none');
    hold on; grid on;

    [max_capture_time, max_idx] = max(capture_times(scenario, :));
    plot(jink_distances(max_idx), max_capture_time, 'r^', ...
        'MarkerSize', 9, 'MarkerFaceColor', 'r');
    xline(jink_distances(max_idx), 'r--', 'LineWidth', 1.2);

    xlabel('Jink distance (m)');
    ylabel('Capture time (s)');
    title(sprintf('Scenario %d (r_{det} = %.0f m)', ...
        scenario, start_distances(scenario)));
    legend('Capture time', sprintf('Maximum: %.2f m', jink_distances(max_idx)), ...
        'Location', 'best');
    set(gca, 'FontSize', 10, 'Box', 'on');
end

sgtitle('Strategy 1: Capture Time for a Range of Jink Distances');
exportgraphics(gcf, fullfile(plot_folder, 'jink_distance_capture_time.png'), ...
    'Resolution', 200);
close all;

fprintf('Strategy 1 maximum capture times by scenario:\n');
for scenario = 1:5
    [max_capture_time, max_idx] = max(capture_times(scenario, :));
    fprintf('  Scenario %d: %.2f m jink distance -> %.2f s capture time\n', ...
        scenario, jink_distances(max_idx), max_capture_time);
end

function capture_time = simulate_capture_time(r_det, alpha_1, r_jink, ...
    v_p, v_e, R_p, R_e, l_cap, T_max, dt)
    w_p_max = v_p / R_p;
    w_e_max = v_e / R_e;
    N = round(T_max / dt);

    x_E = 0;
    y_E = 0;
    theta_E = 0;
    x_P = -r_det * cos(alpha_1);
    y_P = r_det * sin(alpha_1);
    theta_P = 0;
    capture_time = T_max;
    is_jinking = false;
    theta_target = 0;

    for k = 1:N-1
        dist_P = hypot(x_E - x_P, y_E - y_P);
        if dist_P <= l_cap
            capture_time = (k - 1) * dt;
            return;
        end

        theta_LOS = atan2(y_E - y_P, x_E - x_P);
        dtheta_P = atan2(sin(theta_LOS - theta_P), ...
            cos(theta_LOS - theta_P));
        if abs(dtheta_P) <= w_p_max * dt
            w_P = dtheta_P / dt;
        else
            w_P = w_p_max * sign(dtheta_P);
        end

        if is_jinking && dist_P > r_jink + 1.5
            is_jinking = false;
        end

        if ~is_jinking && dist_P <= r_jink
            is_jinking = true;
            theta_target = theta_LOS + pi / 2;
        end

        if is_jinking
            dtheta_E = atan2(sin(theta_target - theta_E), ...
                cos(theta_target - theta_E));
            if abs(dtheta_E) <= w_e_max * dt
                w_E = dtheta_E / dt;
            else
                w_E = w_e_max * sign(dtheta_E);
            end
        else
            theta_flee = theta_LOS;
            dtheta_E = atan2(sin(theta_flee - theta_E), ...
                cos(theta_flee - theta_E));
            if abs(dtheta_E) <= w_e_max * dt
                w_E = dtheta_E / dt;
            else
                w_E = w_e_max * sign(dtheta_E);
            end
        end

        x_P = x_P + v_p * cos(theta_P) * dt;
        y_P = y_P + v_p * sin(theta_P) * dt;
        theta_P = theta_P + w_P * dt;
        x_E = x_E + v_e * cos(theta_E) * dt;
        y_E = y_E + v_e * sin(theta_E) * dt;
        theta_E = theta_E + w_E * dt;
    end
end