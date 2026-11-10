% pair_pursuit.m
% Part 2: Cooperative Pair Hunting (Pure Pursuit + Lead Pursuit Pincer)
clear; clc; close all;

%% 1. Model Parameters
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)
R_p = 1.5;               % Pursuer min turning radius (m)
R_e = 0.5;               % Evader min turning radius (m)

w_p_max = v_p / R_p;     % Max angular velocity of pursuer (11.11 rad/s)
w_e_max = v_e / R_e;     % Max angular velocity of evader (27.78 rad/s)

l_cap = 1.0;             % Capture radius (m)
r_det = 25.0;            % Initial detection distance (m) [15, 50]
r_jink = 3.5;            % Critical jink distance (m)
tau_max = 0.6;           % Max look-ahead time for P2 lead pursuit (s)
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.005;              % Time step (s)
N = round(T_max / dt);
start_distances = [15, 20, 25, 30, 35];
flank_angles = deg2rad([15, 20, 25, 30, 35]);
plot_folder = fullfile(pwd, 'strategy_2_plots');
if ~exist(plot_folder, 'dir')
    mkdir(plot_folder);
end

for scenario = 1:5
r_det = start_distances(scenario);
%% 2. Initial Conditions (t = 0)
alpha = flank_angles(scenario);

% Preallocate state arrays
x_E  = zeros(1, N); y_E  = zeros(1, N); theta_E  = zeros(1, N);
x_P1 = zeros(1, N); y_P1 = zeros(1, N); theta_P1 = zeros(1, N);
x_P2 = zeros(1, N); y_P2 = zeros(1, N); theta_P2 = zeros(1, N);
d1 = zeros(1, N); d2 = zeros(1, N);
t_vec = (0:N-1) * dt;

% Evader starts at origin facing +x
x_E(1) = 0; y_E(1) = 0; theta_E(1) = 0;

% Pursuer 1 (Top/Left Flank) & Pursuer 2 (Bottom/Right Flank)
x_P1(1) = -r_det * cos(alpha);  y_P1(1) =  r_det * sin(alpha);  theta_P1(1) = 0;
x_P2(1) = -r_det * cos(-alpha); y_P2(1) =  r_det * sin(-alpha); theta_P2(1) = 0;

captured = false;
end_idx = N;

%% 3. Time-Stepping Simulation Loop (Forward Euler)
for k = 1:N-1
    d1(k) = hypot(x_E(k) - x_P1(k), y_E(k) - y_P1(k));
    d2(k) = hypot(x_E(k) - x_P2(k), y_E(k) - y_P2(k));
    
    if min(d1(k), d2(k)) <= l_cap
        captured = true;
        end_idx = k;
        break;
    end
    
    % --- Pursuer 1 Control: Pure Pursuit (Direct Line-of-Sight) ---
    theta_LOS1 = atan2(y_E(k) - y_P1(k), x_E(k) - x_P1(k));
    dtheta_P1 = atan2(sin(theta_LOS1 - theta_P1(k)), cos(theta_LOS1 - theta_P1(k)));
    if abs(dtheta_P1) <= w_p_max * dt
        w_P1 = dtheta_P1 / dt;
    else
        w_P1 = w_p_max * sign(dtheta_P1);
    end
    
    % --- Pursuer 2 Control: Cooperative Lead Pursuit ---
    tau = min(tau_max, d2(k) / v_p);
    x_lead = x_E(k) + tau * v_e * cos(theta_E(k));
    y_lead = y_E(k) + tau * v_e * sin(theta_E(k));
    
    theta_LOS2 = atan2(y_lead - y_P2(k), x_lead - x_P2(k));
    dtheta_P2 = atan2(sin(theta_LOS2 - theta_P2(k)), cos(theta_LOS2 - theta_P2(k)));
    if abs(dtheta_P2) <= w_p_max * dt
        w_P2 = dtheta_P2 / dt;
    else
        w_P2 = w_p_max * sign(dtheta_P2);
    end
    
    % --- Evader Control: Evade Closest Threat ---
    if d1(k) <= r_jink && d1(k) <= d2(k)
        w_E = -w_e_max;
    elseif d2(k) <= r_jink && d2(k) < d1(k)
        w_E = w_e_max;
    else
        vx_threat = (x_E(k) - x_P1(k))/d1(k) + (x_E(k) - x_P2(k))/d2(k);
        vy_threat = (y_E(k) - y_P1(k))/d1(k) + (y_E(k) - y_P2(k))/d2(k);
        theta_flee = atan2(vy_threat, vx_threat);
        dtheta_E = atan2(sin(theta_flee - theta_E(k)), cos(theta_flee - theta_E(k)));
        if abs(dtheta_E) <= w_e_max * dt
            w_E = dtheta_E / dt;
        else
            w_E = w_e_max * sign(dtheta_E);
        end
    end
    
    % --- Euler Kinematic Updates ---
    x_P1(k+1) = x_P1(k) + v_p * cos(theta_P1(k)) * dt;
    y_P1(k+1) = y_P1(k) + v_p * sin(theta_P1(k)) * dt;
    theta_P1(k+1) = theta_P1(k) + w_P1 * dt;
    
    x_P2(k+1) = x_P2(k) + v_p * cos(theta_P2(k)) * dt;
    y_P2(k+1) = y_P2(k) + v_p * sin(theta_P2(k)) * dt;
    theta_P2(k+1) = theta_P2(k) + w_P2 * dt;
    
    x_E(k+1) = x_E(k) + v_e * cos(theta_E(k)) * dt;
    y_E(k+1) = y_E(k) + v_e * sin(theta_E(k)) * dt;
    theta_E(k+1) = theta_E(k) + w_E * dt;
end

d1(end_idx) = hypot(x_E(end_idx) - x_P1(end_idx), y_E(end_idx) - y_P1(end_idx));
d2(end_idx) = hypot(x_E(end_idx) - x_P2(end_idx), y_E(end_idx) - y_P2(end_idx));

%% 4. Figure 1: Combined 2D Trajectory Plot (All 3 Players on Same Axis)
figure('Color', 'w', 'Position', [80, 80, 1050, 700]);
hold on; grid on; axis equal;

% Plot all 3 trajectories on the same axis
plot(x_E(1:end_idx),  y_E(1:end_idx),  'b-',  'LineWidth', 2.2, 'DisplayName', 'Evader (E)');
plot(x_P1(1:end_idx), y_P1(1:end_idx), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Raptor 1 (P_1 - Pure Pursuit)');
plot(x_P2(1:end_idx), y_P2(1:end_idx), 'g-.', 'LineWidth', 2.0, 'DisplayName', 'Raptor 2 (P_2 - Lead Pursuit)');

% Plot Start Positions (t = 0)
plot(x_E(1),  y_E(1),  'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
plot(x_P1(1), y_P1(1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(x_P2(1), y_P2(1), 'gd', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');

% Plot Final/Capture Positions
plot(x_E(end_idx),  y_E(end_idx),  'ko', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'y', 'DisplayName', 'Capture Point');
plot(x_P1(end_idx), y_P1(end_idx), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(x_P2(end_idx), y_P2(end_idx), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');

% Plot Capture Radius Circle around Evader's Final Position
th_circ = linspace(0, 2*pi, 100);
plot(x_E(end_idx) + l_cap*cos(th_circ), y_E(end_idx) + l_cap*sin(th_circ), ...
    'm--', 'LineWidth', 2.2, 'DisplayName', 'Capture Radius (1.0 m)');

% Plot Heading Arrows along the Path
arrow_end = max(1, end_idx - round(0.5 / dt));
if arrow_end > 1
    idx_arrows = round(linspace(1, arrow_end, 5));
    quiver(x_E(idx_arrows),  y_E(idx_arrows),  cos(theta_E(idx_arrows)),  sin(theta_E(idx_arrows)),  0.35, 'b', 'LineWidth', 1.1, 'HandleVisibility', 'off');
    quiver(x_P1(idx_arrows), y_P1(idx_arrows), cos(theta_P1(idx_arrows)), sin(theta_P1(idx_arrows)), 0.35, 'r', 'LineWidth', 1.1, 'HandleVisibility', 'off');
    quiver(x_P2(idx_arrows), y_P2(idx_arrows), cos(theta_P2(idx_arrows)), sin(theta_P2(idx_arrows)), 0.35, 'g', 'LineWidth', 1.1, 'HandleVisibility', 'off');
end

set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
xlabel('Horizontal Position x (m)', 'FontSize', 13);
ylabel('Vertical Position y (m)', 'FontSize', 13);
title(sprintf('Part 2: Pair Hunting Trajectories (r_{det} = %.0f m, t_{cap} = %.2f s)', ...
    r_det, t_vec(end_idx)), 'FontSize', 14);
legend('Location', 'best');
exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_trajectory.png', scenario)), 'Resolution', 200);

%% 5. Figure 2: Separation Distance Plot
figure('Color', 'w', 'Position', [180, 120, 850, 550]);
hold on; grid on;
plot(t_vec(1:end_idx), d1(1:end_idx), 'r-', 'LineWidth', 1.8, 'DisplayName', 'd_1(t) (Raptor 1)');
plot(t_vec(1:end_idx), d2(1:end_idx), 'g-', 'LineWidth', 1.8, 'DisplayName', 'd_2(t) (Raptor 2)');
yline(l_cap, 'k--', 'Capture Threshold (l = 1.0 m)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
xlabel('Time t (s)', 'FontSize', 13);
ylabel('Distance to Prey (m)', 'FontSize', 13);
title('Part 2: Pair Separation Distances vs. Time', 'FontSize', 14);
legend('Location', 'southoutside', 'NumColumns', 2);
exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_separation.png', scenario)), 'Resolution', 200);
close all;
end