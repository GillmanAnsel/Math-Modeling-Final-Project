% strategy_4_voronoi_pursuit.m
% Part 2: Voronoi Centroid Escape & OA-ECBVC Cooperative Pursuit (Wang et al., 2023)
clear; clc; close all;

%% 1. Model Parameters and Output Directories
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)
R_p = 1.5;               % Pursuer min turning radius (m) [Set to 5.0 for realistic regime]
R_e = 0.5;               % Evader min turning radius (m)
w_p_max = v_p / R_p;     % Max angular velocity of pursuer (rad/s)
w_e_max = v_e / R_e;     % Max angular velocity of evader (rad/s)
l_cap = 1.0;             % Capture radius (m)
alpha = deg2rad(25);     % Initial flanking angle (rad)
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.01;               % Time step (s)
N = round(T_max / dt);

start_distances = [15, 20, 25, 30, 35];

plot_folder = 'C:\Users\agillman5916\Documents\strategy_4_voronoi\plots';
animation_folder = 'C:\Users\agillman5916\Documents\strategy_4_voronoi\animations';

if ~exist(plot_folder, 'dir')
    mkdir(plot_folder);
end
if ~exist(animation_folder, 'dir')
    mkdir(animation_folder);
end

%% Iterate over detection distances
for scenario = 1:numel(start_distances)
    r_det = start_distances(scenario);
    fprintf('Running Voronoi Pursuit for scenario %d (r_det = %.0f m)...\n', scenario, r_det);
    
    %% 2. Initial Conditions (t = 0)
    x_E  = zeros(1, N); y_E  = zeros(1, N); theta_E  = zeros(1, N);
    x_P1 = zeros(1, N); y_P1 = zeros(1, N); theta_P1 = zeros(1, N);
    x_P2 = zeros(1, N); y_P2 = zeros(1, N); theta_P2 = zeros(1, N);
    d1   = zeros(1, N); d2   = zeros(1, N);
    t_vec = (0:N-1) * dt;
    
    % Store Voronoi polygons and centroids for animation
    V_E_hist  = cell(1, N); V_P1_hist = cell(1, N); V_P2_hist = cell(1, N);
    box_hist  = cell(1, N);
    C_E_hist  = zeros(2, N); C_P1_hist = zeros(2, N); C_P2_hist = zeros(2, N);
    
    % Initial states
    x_E(1) = 0; y_E(1) = 0; theta_E(1) = 0;
    x_P1(1) = -r_det * cos(alpha);  y_P1(1) =  r_det * sin(alpha);  theta_P1(1) = 0;
    x_P2(1) = -r_det * cos(-alpha); y_P2(1) =  r_det * sin(-alpha); theta_P2(1) = 0;
    
    % Initial bounding box half-lengths with buffer margin
    margin = 4.0;
    Dx = max(abs([x_P1(1) - x_E(1), x_P2(1) - x_E(1)])) + margin;
    Dy = max(abs([y_P1(1) - y_E(1), y_P2(1) - y_E(1)])) + margin;
    
    captured = false;
    end_idx = N;
    
    %% 3. Time-Stepping Simulation Loop
    for k = 1:N-1
        u_E  = [x_E(k);  y_E(k)];
        u_P1 = [x_P1(k); y_P1(k)];
        u_P2 = [x_P2(k); y_P2(k)];
        
        d1(k) = norm(u_E - u_P1);
        d2(k) = norm(u_E - u_P2);
        
        if min(d1(k), d2(k)) <= l_cap
            captured = true;
            end_idx = k;
            break;
        end
        
        % Update Evader-Centered Bounding Box E(t)
        min_span_x = max(abs([u_P1(1) - u_E(1), u_P2(1) - u_E(1)])) + 2.0;
        min_span_y = max(abs([u_P1(2) - u_E(2), u_P2(2) - u_E(2)])) + 2.0;
        Dx = max(min_span_x, Dx - 0.5 * v_p * dt);
        Dy = max(min_span_y, Dy - 0.5 * v_p * dt);
        
        box_poly = [u_E(1)-Dx, u_E(2)-Dy;
                    u_E(1)+Dx, u_E(2)-Dy;
                    u_E(1)+Dx, u_E(2)+Dy;
                    u_E(1)-Dx, u_E(2)+Dy];
        
        % Construct Bounded Voronoi Cells
        V_E = clip_halfplane(box_poly, u_E, u_P1);
        V_E = clip_halfplane(V_E,      u_E, u_P2);
        
        V_P1 = clip_halfplane(box_poly, u_P1, u_E);
        V_P1 = clip_halfplane(V_P1,     u_P1, u_P2);
        
        V_P2 = clip_halfplane(box_poly, u_P2, u_E);
        V_P2 = clip_halfplane(V_P2,     u_P2, u_P1);
        
        % Compute Centroids
        C_E  = poly_centroid(V_E,  u_E);
        C_P1 = poly_centroid(V_P1, u_P1);
        C_P2 = poly_centroid(V_P2, u_P2);
        
        % Save geometry for animation
        box_hist{k}  = box_poly;
        V_E_hist{k}  = V_E;  C_E_hist(:, k)  = C_E;
        V_P1_hist{k} = V_P1; C_P1_hist(:, k) = C_P1;
        V_P2_hist{k} = V_P2; C_P2_hist(:, k) = C_P2;
        
        % Pursuer 1 Control
        theta_vor_P1 = atan2(C_P1(2) - u_P1(2), C_P1(1) - u_P1(1));
        dtheta_P1 = atan2(sin(theta_vor_P1 - theta_P1(k)), cos(theta_vor_P1 - theta_P1(k)));
        if abs(dtheta_P1) <= w_p_max * dt
            w_P1 = dtheta_P1 / dt;
        else
            w_P1 = w_p_max * sign(dtheta_P1);
        end
        
        % Pursuer 2 Control
        theta_vor_P2 = atan2(C_P2(2) - u_P2(2), C_P2(1) - u_P2(1));
        dtheta_P2 = atan2(sin(theta_vor_P2 - theta_P2(k)), cos(theta_vor_P2 - theta_P2(k)));
        if abs(dtheta_P2) <= w_p_max * dt
            w_P2 = dtheta_P2 / dt;
        else
            w_P2 = w_p_max * sign(dtheta_P2);
        end
        
        % Evader Control
        theta_vor_E = atan2(C_E(2) - u_E(2), C_E(1) - u_E(1));
        dtheta_E = atan2(sin(theta_vor_E - theta_E(k)), cos(theta_vor_E - theta_E(k)));
        if abs(dtheta_E) <= w_e_max * dt
            w_E = dtheta_E / dt;
        else
            w_E = w_e_max * sign(dtheta_E);
        end
        
        % Kinematics
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
    
    % Populate final step geometry
    d1(end_idx) = hypot(x_E(end_idx) - x_P1(end_idx), y_E(end_idx) - y_P1(end_idx));
    d2(end_idx) = hypot(x_E(end_idx) - x_P2(end_idx), y_E(end_idx) - y_P2(end_idx));
    box_hist{end_idx}  = box_hist{max(1, end_idx-1)};
    V_E_hist{end_idx}  = V_E_hist{max(1, end_idx-1)};
    V_P1_hist{end_idx} = V_P1_hist{max(1, end_idx-1)};
    V_P2_hist{end_idx} = V_P2_hist{max(1, end_idx-1)};
    C_E_hist(:, end_idx)  = C_E_hist(:, max(1, end_idx-1));
    C_P1_hist(:, end_idx) = C_P1_hist(:, max(1, end_idx-1));
    C_P2_hist(:, end_idx) = C_P2_hist(:, max(1, end_idx-1));
    
    %% 4. Static 2D Trajectory Plot
    figure('Color', 'w', 'Position', [80, 80, 1000, 680], 'Visible', 'off');
    hold on; grid on; axis equal;
    plot(x_E(1:end_idx),  y_E(1:end_idx),  'b-',  'LineWidth', 2.2, 'DisplayName', 'Evader (Voronoi Escape)');
    plot(x_P1(1:end_idx), y_P1(1:end_idx), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Raptor 1 (P_1 - Voronoi Pursuit)');
    plot(x_P2(1:end_idx), y_P2(1:end_idx), 'g-.', 'LineWidth', 2.0, 'DisplayName', 'Raptor 2 (P_2 - Voronoi Pursuit)');
    plot(x_E(1),  y_E(1),  'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
    plot(x_P1(1), y_P1(1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    plot(x_P2(1), y_P2(1), 'gd', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
    plot(x_E(end_idx),  y_E(end_idx),  'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'DisplayName', 'Capture / End Point');
    plot(x_P1(end_idx), y_P1(end_idx), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    plot(x_P2(end_idx), y_P2(end_idx), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
    
    th_circ = linspace(0, 2*pi, 100);
    plot(x_E(end_idx) + l_cap*cos(th_circ), y_E(end_idx) + l_cap*sin(th_circ), ...
        'm--', 'LineWidth', 2.0, 'DisplayName', 'Capture Radius (1.0 m)');
    quiver(x_E(end_idx), y_E(end_idx), cos(theta_E(end_idx)), sin(theta_E(end_idx)), ...
        1.5, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
    quiver(x_P1(end_idx), y_P1(end_idx), cos(theta_P1(end_idx)), sin(theta_P1(end_idx)), ...
        1.5, 'r', 'LineWidth', 1.8, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
    quiver(x_P2(end_idx), y_P2(end_idx), cos(theta_P2(end_idx)), sin(theta_P2(end_idx)), ...
        1.5, 'g', 'LineWidth', 1.8, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
    
    set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
    xlabel('Horizontal Position x (m)', 'FontSize', 13);
    ylabel('Vertical Position y (m)', 'FontSize', 13);
    title(sprintf('Voronoi Strategy Trajectories (Scenario %d: r_{det} = %.0f m, t_{final} = %.2f s)', ...
        scenario, r_det, t_vec(end_idx)), 'FontSize', 14);
    legend('Location', 'best');
    
    exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_voronoi_trajectory.png', scenario)), 'Resolution', 200);
    
    %% 5. Static Separation Distance Plot
    figure('Color', 'w', 'Position', [160, 120, 850, 520], 'Visible', 'off');
    hold on; grid on;
    plot(t_vec(1:end_idx), d1(1:end_idx), 'r-', 'LineWidth', 1.8, 'DisplayName', 'd_1(t) (Raptor 1)');
    plot(t_vec(1:end_idx), d2(1:end_idx), 'g-', 'LineWidth', 1.8, 'DisplayName', 'd_2(t) (Raptor 2)');
    yline(l_cap, 'k--', 'Capture Threshold (l = 1.0 m)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    set(gca, 'FontSize', 12, 'LineWidth', 1.0, 'Box', 'on');
    xlabel('Time t (s)', 'FontSize', 13);
    ylabel('Distance to Prey (m)', 'FontSize', 13);
    title(sprintf('Voronoi Strategy Separation (Scenario %d: r_{det} = %.0f m)', ...
        scenario, r_det), 'FontSize', 14);
    legend('Location', 'northeast');
    
    exportgraphics(gcf, fullfile(plot_folder, sprintf('scenario_%d_voronoi_separation.png', scenario)), 'Resolution', 200);
    
    %% 6. Animation Setup and Video Write
    fig_anim = figure('Color', 'w', 'Position', [100, 100, 1100, 750], 'Visible', 'off');
    ax = axes('Parent', fig_anim);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    set(ax, 'FontSize', 12, 'Box', 'on');
    xlabel(ax, 'Horizontal Position x (m)', 'FontSize', 13);
    ylabel(ax, 'Vertical Position y (m)', 'FontSize', 13);
    
    all_x = [x_E(1:end_idx), x_P1(1:end_idx), x_P2(1:end_idx)];
    all_y = [y_E(1:end_idx), y_P1(1:end_idx), y_P2(1:end_idx)];
    pad = r_det + 8;
    xlim(ax, [min(all_x) - pad/2, max(all_x) + pad]);
    ylim(ax, [min(all_y) - pad,   max(all_y) + pad]);
    
    h_VE  = patch(ax, NaN, NaN, [0.75, 0.88, 1.00], 'FaceAlpha', 0.45, 'EdgeColor', 'b', 'LineWidth', 1.5, 'DisplayName', 'Evader Cell V_E^E');
    h_VP1 = patch(ax, NaN, NaN, [1.00, 0.80, 0.80], 'FaceAlpha', 0.45, 'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'Raptor 1 Cell V_{P1}^E');
    h_VP2 = patch(ax, NaN, NaN, [0.80, 0.95, 0.80], 'FaceAlpha', 0.45, 'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.5, 'DisplayName', 'Raptor 2 Cell V_{P2}^E');
    h_box = plot(ax, NaN, NaN, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Bounding Box E(t)');
    
    h_trE  = plot(ax, NaN, NaN, 'b-',  'LineWidth', 2.0, 'HandleVisibility', 'off');
    h_trP1 = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1.8, 'HandleVisibility', 'off');
    h_trP2 = plot(ax, NaN, NaN, 'g-.', 'LineWidth', 1.8, 'HandleVisibility', 'off');
    
    h_posE  = plot(ax, NaN, NaN, 'bo', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'DisplayName', 'Evader u_E(t)');
    h_posP1 = plot(ax, NaN, NaN, 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r', 'DisplayName', 'Raptor 1 u_{P1}(t)');
    h_posP2 = plot(ax, NaN, NaN, 'gd', 'MarkerSize', 9, 'MarkerFaceColor', [0, 0.6, 0], 'DisplayName', 'Raptor 2 u_{P2}(t)');
    
    h_CE  = plot(ax, NaN, NaN, 'b*', 'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', 'Centroid C_{VE}');
    h_CP1 = plot(ax, NaN, NaN, 'r*', 'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', 'Centroid C_{VP1}');
    h_CP2 = plot(ax, NaN, NaN, 'g*', 'MarkerSize', 9, 'LineWidth', 1.5, 'Color', [0, 0.6, 0], 'DisplayName', 'Centroid C_{VP2}');
    
    legend(ax, 'Location', 'northeastoutside');
    title_handle = title(ax, '');
    
    video_path = fullfile(animation_folder, sprintf('scenario_%d_voronoi_animation.mp4', scenario));
    vw = VideoWriter(video_path, 'MPEG-4');
    vw.FrameRate = 30;
    open(vw);
    
    frame_step = 4;
    for k = 1:frame_step:end_idx
        if ~isempty(V_E_hist{k})
            set(h_VE, 'XData', V_E_hist{k}(:,1), 'YData', V_E_hist{k}(:,2));
        end
        if ~isempty(V_P1_hist{k})
            set(h_VP1, 'XData', V_P1_hist{k}(:,1), 'YData', V_P1_hist{k}(:,2));
        end
        if ~isempty(V_P2_hist{k})
            set(h_VP2, 'XData', V_P2_hist{k}(:,1), 'YData', V_P2_hist{k}(:,2));
        end
        
        b_poly = [box_hist{k}; box_hist{k}(1,:)];
        set(h_box, 'XData', b_poly(:,1), 'YData', b_poly(:,2));
        
        set(h_trE,  'XData', x_E(1:k),  'YData', y_E(1:k));
        set(h_trP1, 'XData', x_P1(1:k), 'YData', y_P1(1:k));
        set(h_trP2, 'XData', x_P2(1:k), 'YData', y_P2(1:k));
        
        set(h_posE,  'XData', x_E(k),  'YData', y_E(k));
        set(h_posP1, 'XData', x_P1(k), 'YData', y_P1(k));
        set(h_posP2, 'XData', x_P2(k), 'YData', y_P2(k));
        
        set(h_CE,  'XData', C_E_hist(1,k),  'YData', C_E_hist(2,k));
        set(h_CP1, 'XData', C_P1_hist(1,k), 'YData', C_P1_hist(2,k));
        set(h_CP2, 'XData', C_P2_hist(1,k), 'YData', C_P2_hist(2,k));
        
        set(title_handle, 'String', sprintf('Scenario %d Voronoi ECBVC (t = %.2f s, min d_i = %.2f m)', ...
            scenario, t_vec(k), min(d1(k), d2(k))));
        
        drawnow;
        writeVideo(vw, getframe(fig_anim));
    end
    close(vw);
    close all; % Clean up figures before next iteration
end
fprintf('All %d scenarios processed and saved successfully.\n', numel(start_distances));

%% ========================================================================
%% Helper Functions: Half-Plane Clipping & Polygon Centroid
%% ========================================================================
function poly_out = clip_halfplane(poly_in, p_keep, p_other)
    if isempty(poly_in)
        poly_out = [];
        return;
    end
    a = p_other - p_keep;
    b = dot(a, 0.5 * (p_keep + p_other));
    
    poly_out = [];
    num_verts = size(poly_in, 1);
    for i = 1:num_verts
        curr_pt = poly_in(i, :)';
        prev_pt = poly_in(mod(i - 2, num_verts) + 1, :)';
        
        curr_in = (dot(a, curr_pt) <= b + 1e-9);
        prev_in = (dot(a, prev_pt) <= b + 1e-9);
        
        if curr_in
            if ~prev_in
                inter = intersect_bisector(prev_pt, curr_pt, a, b);
                poly_out = [poly_out; inter']; %#ok<AGROW>
            end
            poly_out = [poly_out; curr_pt']; %#ok<AGROW>
        elseif prev_in
            inter = intersect_bisector(prev_pt, curr_pt, a, b);
            poly_out = [poly_out; inter']; %#ok<AGROW>
        end
    end
end

function pt = intersect_bisector(p1, p2, a, b)
    d = p2 - p1;
    denom = dot(a, d);
    if abs(denom) < 1e-12
        pt = p1;
    else
        t = (b - dot(a, p1)) / denom;
        t = max(0, min(1, t));
        pt = p1 + t * d;
    end
end

function C = poly_centroid(V, fallback_pos)
    if size(V, 1) < 3
        C = fallback_pos;
        return;
    end
    x = V(:, 1);
    y = V(:, 2);
    x_next = [x(2:end); x(1)];
    y_next = [y(2:end); y(1)];
    
    cross_term = (x .* y_next) - (x_next .* y);
    A = 0.5 * sum(cross_term);
    
    if abs(A) < 1e-7
        C = mean(V, 1)';
    else
        Cx = sum((x + x_next) .* cross_term) / (6 * A);
        Cy = sum((y + y_next) .* cross_term) / (6 * A);
        C = [Cx; Cy];
    end
end
