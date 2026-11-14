% monte_carlo_capture_rate_analysis.m
% Monte Carlo simulation evaluating and comparing average capture rates for:
%   Part 1: Solitary Hunting (1.26x G-force ratio, benchmarked against Lions/Cheetahs)
%   Part 2: Pair Pure Pursuit (1.85x G-force ratio, benchmarked against Wolf Packs)
%   Part 3: Pair Voronoi OA-ECBVC (1.85x G-force ratio, benchmarked against Wolf Packs)
clear; clc;
rng(42); % Fixed seed for reproducible report statistics

%% 1. Shared Kinematic Parameters
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)
l_cap = 1.0;             % Capture radius (1.0 m across all strategies)

num_trials = 1000;
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.01;               % Time step (s)
N = round(T_max / dt);
jink_angle = deg2rad(110); % Optimal break-turn angle

% Pre-generate randomized encounter conditions (shared across all regimes)
% 1. Detection distance: Uniformly distributed between 15m and 50m
r_det_array = 15 + (50 - 15) * rand(1, num_trials);
% 2. Solitary attack angle: Uniformly distributed between -45 and 45 deg
alpha_solitary = deg2rad(-45 + 90 * rand(1, num_trials));
% 3. Pair flanking half-angle: Uniformly distributed between 15 and 45 deg
alpha_pair = deg2rad(15 + (45 - 15) * rand(1, num_trials));
% 4. Prey Jink Trigger: Uniformly distributed between 2.0m and 5.0m
r_jink_array = 2.0 + (5.0 - 2.0) * rand(1, num_trials);

%% ========================================================================
%% PART 1: Solitary Pursuit Regime (1.26x G-Force Ratio -> Lion/Cheetah)
%% ========================================================================
R_p_sol = 4.0;           % Pursuer min turning radius (m)
R_e_sol = 3.5;           % Evader min turning radius (m)
w_p_sol = v_p / R_p_sol;
w_e_sol = v_e / R_e_sol;

g_p_sol = ((v_p^2) / R_p_sol) / 9.81; % 7.08g
g_e_sol = ((v_e^2) / R_e_sol) / 9.81; % 5.62g

fprintf('=========================================================\n');
fprintf('PART 1: SOLITARY PURSUIT BIOMECHANICAL VALIDATION\n');
fprintf('=========================================================\n');
fprintf('Capture Radius l_cap:                %.2f m\n', l_cap);
fprintf('Predator max G-force (R_p = %.2f m): %.2fg\n', R_p_sol, g_p_sol);
fprintf('Prey max G-force     (R_e = %.2f m): %.2fg\n', R_e_sol, g_e_sol);
fprintf('Predator/Prey G-Force Ratio:         %.2fx\n\n', g_p_sol / g_e_sol);

captures_solitary = 0;
fprintf('Running %d solitary pursuit trials...\n', num_trials);

for trial = 1:num_trials
    r_det  = r_det_array(trial);
    alpha  = alpha_solitary(trial);
    r_jink = r_jink_array(trial);
    
    x_E = 0; y_E = 0; theta_E = 0;
    x_P = -r_det * cos(alpha);
    y_P =  r_det * sin(alpha);
    theta_P = 0;
    
    is_jinking = false;
    theta_target = 0;
    
    for k = 1:N
        dist_P = hypot(x_E - x_P, y_E - y_P);
        
        if dist_P <= l_cap
            captures_solitary = captures_solitary + 1;
            break;
        end
        
        % Pursuer Control (Pure Pursuit)
        theta_LOS = atan2(y_E - y_P, x_E - x_P);
        dtheta_P = atan2(sin(theta_LOS - theta_P), cos(theta_LOS - theta_P));
        w_P = sign(dtheta_P) * min(abs(dtheta_P)/dt, w_p_sol);
        
        % Evader Control (With 4.0m reset buffer)
        if is_jinking && dist_P > r_jink + 4.0
            is_jinking = false;
        end
        
        if ~is_jinking && dist_P <= r_jink
            is_jinking = true;
            theta_target = theta_LOS + jink_angle; 
        end
        
        if is_jinking
            dtheta_E = atan2(sin(theta_target - theta_E), cos(theta_target - theta_E));
        else
            dtheta_E = atan2(sin(theta_LOS - theta_E), cos(theta_LOS - theta_E));
        end
        w_E = sign(dtheta_E) * min(abs(dtheta_E)/dt, w_e_sol);
        
        % Kinematic Update
        x_P = x_P + v_p * cos(theta_P) * dt;
        y_P = y_P + v_p * sin(theta_P) * dt;
        theta_P = theta_P + w_P * dt;
        
        x_E = x_E + v_e * cos(theta_E) * dt;
        y_E = y_E + v_e * sin(theta_E) * dt;
        theta_E = theta_E + w_E * dt;
    end
end

rate_solitary = (captures_solitary / num_trials) * 100;
fprintf('Solitary Captures: %d / %d (%.1f%%)\n\n', ...
    captures_solitary, num_trials, rate_solitary);

%% ========================================================================
%% PART 2: Pair Pursuit Regime (1.85x G-Force Ratio -> Wolf-Elk Benchmark)
%% ========================================================================
R_p_pair = 4.0;          % Pursuer min turning radius (m)
R_e_pair = 5.14;         % Evader min turning radius (m) for 1.85x ratio
w_p_pair = v_p / R_p_pair;
w_e_pair = v_e / R_e_pair;

g_p_pair = ((v_p^2) / R_p_pair) / 9.81; % 7.08g
g_e_pair = ((v_e^2) / R_e_pair) / 9.81; % 3.83g

fprintf('=========================================================\n');
fprintf('PART 2: PAIR PURE PURSUIT VALIDATION (1.85x WOLF RATIO)\n');
fprintf('=========================================================\n');
fprintf('Capture Radius l_cap:                %.2f m\n', l_cap);
fprintf('Predator max G-force (R_p = %.2f m): %.2fg\n', R_p_pair, g_p_pair);
fprintf('Prey max G-force     (R_e = %.2f m): %.2fg\n', R_e_pair, g_e_pair);
fprintf('Predator/Prey G-Force Ratio:         %.2fx (Target: 1.85x)\n\n', g_p_pair / g_e_pair);

captures_pair = 0;
fprintf('Running %d cooperative pair pursuit trials...\n', num_trials);

for trial = 1:num_trials
    r_det  = r_det_array(trial);
    alpha  = alpha_pair(trial);
    r_jink = r_jink_array(trial);
    
    x_E = 0; y_E = 0; theta_E = 0;
    
    % Symmetric flanking start positions for P1 (+alpha) and P2 (-alpha)
    x_P1 = -r_det * cos(alpha);  y_P1 =  r_det * sin(alpha);  theta_P1 = 0;
    x_P2 = -r_det * cos(alpha);  y_P2 = -r_det * sin(alpha);  theta_P2 = 0;
    
    is_jinking = false;
    theta_target = 0;
    
    for k = 1:N
        d1 = hypot(x_E - x_P1, y_E - y_P1);
        d2 = hypot(x_E - x_P2, y_E - y_P2);
        min_d = min(d1, d2);
        
        if min_d <= l_cap
            captures_pair = captures_pair + 1;
            break;
        end
        
        % Pursuer 1 & 2 Control (Dual Pure Pursuit)
        theta_LOS1 = atan2(y_E - y_P1, x_E - x_P1);
        theta_LOS2 = atan2(y_E - y_P2, x_E - x_P2);
        
        dtheta_P1 = atan2(sin(theta_LOS1 - theta_P1), cos(theta_LOS1 - theta_P1));
        dtheta_P2 = atan2(sin(theta_LOS2 - theta_P2), cos(theta_LOS2 - theta_P2));
        
        w_P1 = sign(dtheta_P1) * min(abs(dtheta_P1)/dt, w_p_pair);
        w_P2 = sign(dtheta_P2) * min(abs(dtheta_P2)/dt, w_p_pair);
        
        % Evader Control against Two Pursuers
        if is_jinking && min_d > r_jink + 4.0
            is_jinking = false;
        end
        
        if ~is_jinking && min_d <= r_jink
            is_jinking = true;
            if d1 <= d2
                theta_target = theta_LOS1 - jink_angle;
            else
                theta_target = theta_LOS2 + jink_angle;
            end
        end
        
        if is_jinking
            dtheta_E = atan2(sin(theta_target - theta_E), cos(theta_target - theta_E));
        else
            vx_flee = (x_E - x_P1)/d1 + (x_E - x_P2)/d2;
            vy_flee = (y_E - y_P1)/d1 + (y_E - y_P2)/d2;
            theta_flee = atan2(vy_flee, vx_flee);
            dtheta_E = atan2(sin(theta_flee - theta_E), cos(theta_flee - theta_E));
        end
        w_E = sign(dtheta_E) * min(abs(dtheta_E)/dt, w_e_pair);
        
        % Kinematic Update
        x_P1 = x_P1 + v_p * cos(theta_P1) * dt;
        y_P1 = y_P1 + v_p * sin(theta_P1) * dt;
        theta_P1 = theta_P1 + w_P1 * dt;
        
        x_P2 = x_P2 + v_p * cos(theta_P2) * dt;
        y_P2 = y_P2 + v_p * sin(theta_P2) * dt;
        theta_P2 = theta_P2 + w_P2 * dt;
        
        x_E = x_E + v_e * cos(theta_E) * dt;
        y_E = y_E + v_e * sin(theta_E) * dt;
        theta_E = theta_E + w_E * dt;
    end
end

rate_pair = (captures_pair / num_trials) * 100;
fprintf('Pair Pure Pursuit Captures: %d / %d (%.1f%%)\n\n', ...
    captures_pair, num_trials, rate_pair);

%% ========================================================================
%% PART 3: Pair Voronoi OA-ECBVC Regime (1.85x Wolf Ratio, 2v1)
%% ========================================================================
fprintf('=========================================================\n');
fprintf('PART 3: PAIR VORONOI (OA-ECBVC) VALIDATION (1.85x RATIO)\n');
fprintf('=========================================================\n');
fprintf('Capture Radius l_cap:                %.2f m\n', l_cap);
fprintf('Predator max G-force (R_p = %.2f m): %.2fg\n', R_p_pair, g_p_pair);
fprintf('Prey max G-force     (R_e = %.2f m): %.2fg\n', R_e_pair, g_e_pair);
fprintf('Predator/Prey G-Force Ratio:         %.2fx (Target: 1.85x)\n\n', g_p_pair / g_e_pair);

captures_voronoi = 0;
fprintf('Running %d cooperative Voronoi pursuit trials...\n', num_trials);

for trial = 1:num_trials
    r_det = r_det_array(trial);
    alpha = alpha_pair(trial);
    
    x_E = 0; y_E = 0; theta_E = 0;
    
    % Symmetric flanking start positions for P1 (+alpha) and P2 (-alpha)
    x_P1 = -r_det * cos(alpha);  y_P1 =  r_det * sin(alpha);  theta_P1 = 0;
    x_P2 = -r_det * cos(alpha);  y_P2 = -r_det * sin(alpha);  theta_P2 = 0;
    
    % Initialize dynamic bounding box half-lengths Dx and Dy
    buffer = 1.5;
    Dx = max(abs([x_P1 - x_E, x_P2 - x_E])) + buffer;
    Dy = max(abs([y_P1 - y_E, y_P2 - y_E])) + buffer;
    
    for k = 1:N
        u_E  = [x_E;  y_E];
        u_P1 = [x_P1; y_P1];
        u_P2 = [x_P2; y_P2];
        
        d1 = hypot(x_E - x_P1, y_E - y_P1);
        d2 = hypot(x_E - x_P2, y_E - y_P2);
        
        if min(d1, d2) <= l_cap
            captures_voronoi = captures_voronoi + 1;
            break;
        end
        
        % Step 2: Compress Bounding Box & Decay Buffer Floor
        buffer = max(0.05, buffer - 0.5 * dt);
        min_span_x = max(abs([x_P1 - x_E, x_P2 - x_E])) + buffer;
        min_span_y = max(abs([y_P1 - y_E, y_P2 - y_E])) + buffer;
        Dx = max(min_span_x, Dx - v_p * dt);
        Dy = max(min_span_y, Dy - v_p * dt);
        
        box_poly = [x_E - Dx, y_E - Dy;
                    x_E + Dx, y_E - Dy;
                    x_E + Dx, y_E + Dy;
                    x_E - Dx, y_E + Dy];
        
        % Construct Bounded Voronoi Cells via Half-Plane Clipping
        V_E  = clip_halfplane(clip_halfplane(box_poly, u_E,  u_P1), u_E,  u_P2);
        V_P1 = clip_halfplane(clip_halfplane(box_poly, u_P1, u_E),  u_P1, u_P2);
        V_P2 = clip_halfplane(clip_halfplane(box_poly, u_P2, u_E),  u_P2, u_P1);
        
        % Compute Centroids C_Vk
        C_E  = poly_centroid(V_E,  u_E);
        C_P1 = poly_centroid(V_P1, u_P1);
        C_P2 = poly_centroid(V_P2, u_P2);
        
        % OA-ECBVC Pursuer Target: Centroid Flanking + Pocket Collapse
        target_P1 = 0.35 * C_P1 + 0.65 * u_E;
        target_P2 = 0.35 * C_P2 + 0.65 * u_E;
        
        % Pursuer 1 Control
        theta_vor_P1 = atan2(target_P1(2) - y_P1, target_P1(1) - x_P1);
        dtheta_P1 = atan2(sin(theta_vor_P1 - theta_P1), cos(theta_vor_P1 - theta_P1));
        w_P1 = sign(dtheta_P1) * min(abs(dtheta_P1)/dt, w_p_pair);
        
        % Pursuer 2 Control
        theta_vor_P2 = atan2(target_P2(2) - y_P2, target_P2(1) - x_P2);
        dtheta_P2 = atan2(sin(theta_vor_P2 - theta_P2), cos(theta_vor_P2 - theta_P2));
        w_P2 = sign(dtheta_P2) * min(abs(dtheta_P2)/dt, w_p_pair);
        
        % Evader Control: Steer to Own Voronoi Centroid C_VE
        theta_vor_E = atan2(C_E(2) - y_E, C_E(1) - x_E);
        dtheta_E = atan2(sin(theta_vor_E - theta_E), cos(theta_vor_E - theta_E));
        w_E = sign(dtheta_E) * min(abs(dtheta_E)/dt, w_e_pair);
        
        % Euler Kinematic Updates
        x_P1 = x_P1 + v_p * cos(theta_P1) * dt;
        y_P1 = y_P1 + v_p * sin(theta_P1) * dt;
        theta_P1 = theta_P1 + w_P1 * dt;
        
        x_P2 = x_P2 + v_p * cos(theta_P2) * dt;
        y_P2 = y_P2 + v_p * sin(theta_P2) * dt;
        theta_P2 = theta_P2 + w_P2 * dt;
        
        x_E = x_E + v_e * cos(theta_E) * dt;
        y_E = y_E + v_e * sin(theta_E) * dt;
        theta_E = theta_E + w_E * dt;
    end
end

rate_voronoi = (captures_voronoi / num_trials) * 100;
fprintf('Pair Voronoi Captures:      %d / %d (%.1f%%)\n\n', ...
    captures_voronoi, num_trials, rate_voronoi);

%% ========================================================================
%% 4. Summary & Empirical Comparison Output
%% ========================================================================
fprintf('=========================================================\n');
fprintf('SUMMARY: SIMULATED VS. EMPIRICAL CAPTURE RATES (l_cap = 1.0 m)\n');
fprintf('=========================================================\n');
fprintf('1. Solitary Pure Pursuit (1.26x G-Ratio): %.1f%%\n', rate_solitary);
fprintf('     -> Empirical Lion (Solitary):        15%% - 25%%\n');
fprintf('     -> Empirical Cheetah (Solitary):     50%% - 58%%\n\n');
fprintf('2. Pair Pure Pursuit (1.85x Wolf Ratio):  %.1f%%\n', rate_pair);
fprintf('3. Pair Voronoi ECBVC (1.85x Wolf Ratio): %.1f%%\n', rate_voronoi);
fprintf('     -> Empirical Wolf Pack (vs. Elk):    ~20%%\n');
fprintf('     -> Empirical Lion Pride (Pack):      30%% - 40%%\n');
fprintf('=========================================================\n');

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
