% sensitivity_analysis_simulations.m
% Comprehensive Sensitivity Analysis across Solitary Pure Pursuit, 
% Pair Pure Pursuit, and Pair Voronoi (OA-ECBVC) Strategies (l_cap = 1.0 m).
clear; clc; close all;

%% 1. Baseline Parameters & Pre-Generated Trial Conditions
rng(42);                 % Fixed seed for exact reproducibility across sweeps
num_trials = 500;        % Trials per parameter step
dt = 0.01;               % Integration time step (s)
l_cap_base = 1.0;        % Standardized capture radius (m)

% Baseline kinematic parameters
vp_base = 60 / 3.6;      % 16.67 m/s
ve_base = 50 / 3.6;      % 13.89 m/s
Rp_base = 4.0;           % Baseline predator turning radius (m)
Re_sol_base  = 3.50;     % Solitary G-ratio prey turning radius (m)
Re_pair_base = 5.14;     % Pair Wolf G-ratio prey turning radius (m)
Re_agile     = 0.50;     % Problem statement agile prey turning radius (m)
Tmax_base = 15.0;        % Baseline stamina limit (s)

% Pre-generate randomized encounter conditions in [0, 1]
u_rdet  = rand(1, num_trials);
u_asol  = rand(1, num_trials);
u_apair = rand(1, num_trials);
u_rjink = rand(1, num_trials);

trials.r_det  = 15 + (50 - 15) * u_rdet;
trials.a_sol  = deg2rad(-45 + 90 * u_asol);
trials.a_pair = deg2rad(15 + (45 - 15) * u_apair);
% Optimal jink radius window [2.0 m, 3.0 m] centered at 2.5 m
trials.r_jink = 2.0 + (3.0 - 2.0) * u_rjink;

%% ========================================================================
%% SWEEP 1: Predator Speed v_p Sensitivity (52 to 68 km/h)
%% ========================================================================
vp_kmh_vec = [52, 56, 60, 64, 68];
rates_vp = zeros(length(vp_kmh_vec), 3);

fprintf('Running Sweep 1: Predator Speed (v_p) Sensitivity...\n');
for i = 1:length(vp_kmh_vec)
    vp_curr = vp_kmh_vec(i) / 3.6;
    [r_sol, r_pair, r_vor] = run_all_strategies(vp_curr, ve_base, ...
        Rp_base, Re_sol_base, Re_pair_base, Tmax_base, l_cap_base, dt, trials);
    rates_vp(i, :) = [r_sol, r_pair, r_vor];
end

%% ========================================================================
%% SWEEP 2: Predator Turning Radius R_p Sensitivity (1.5 to 6.0 m)
%% Evaluates R_p against the agile prey (R_e = 0.5 m) with r_jink in [2, 3] m
%% ========================================================================
Rp_vec = [1.5, 2.5, 3.5, 4.5, 6.0];
rates_Rp = zeros(length(Rp_vec), 3);

fprintf('Running Sweep 2: Predator Turning Radius (R_p) Sensitivity...\n');
for i = 1:length(Rp_vec)
    Rp_curr = Rp_vec(i);
    [r_sol, r_pair, r_vor] = run_all_strategies(vp_base, ve_base, ...
        Rp_curr, Re_agile, Re_agile, Tmax_base, l_cap_base, dt, trials);
    rates_Rp(i, :) = [r_sol, r_pair, r_vor];
end

%% ========================================================================
%% SWEEP 3: Anaerobic Stamina Limit T_max Sensitivity (10.0 to 20.0 s)
%% ========================================================================
Tmax_vec = [10.0, 12.5, 15.0, 17.5, 20.0];
rates_Tmax = zeros(length(Tmax_vec), 3);

fprintf('Running Sweep 3: Stamina Cutoff (T_max) Sensitivity...\n');
for i = 1:length(Tmax_vec)
    Tmax_curr = Tmax_vec(i);
    [r_sol, r_pair, r_vor] = run_all_strategies(vp_base, ve_base, ...
        Rp_base, Re_sol_base, Re_pair_base, Tmax_curr, l_cap_base, dt, trials);
    rates_Tmax(i, :) = [r_sol, r_pair, r_vor];
end

%% ========================================================================
%% SWEEP 4: Local Normalized Sensitivity Index (+10% Perturbations)
%% ========================================================================
fprintf('Running Sweep 4: +10%% Local Elasticity Analysis...\n\n');
[base_sol, base_pair, base_vor] = run_all_strategies(vp_base, ve_base, ...
    3.5, Re_agile, Re_agile, Tmax_base, l_cap_base, dt, trials);
base_vec = [base_sol, base_pair, base_vor];

[vp_p10_s, vp_p10_p, vp_p10_v] = run_all_strategies(1.10*vp_base, ve_base, 3.5, Re_agile, Re_agile, Tmax_base, l_cap_base, dt, trials);
[Rp_p10_s, Rp_p10_p, Rp_p10_v] = run_all_strategies(vp_base, ve_base, 1.10*3.5, Re_agile, Re_agile, Tmax_base, l_cap_base, dt, trials);
[Tm_p10_s, Tm_p10_p, Tm_p10_v] = run_all_strategies(vp_base, ve_base, 3.5, Re_agile, Re_agile, 1.10*Tmax_base, l_cap_base, dt, trials);
[lc_p10_s, lc_p10_p, lc_p10_v] = run_all_strategies(vp_base, ve_base, 3.5, Re_agile, Re_agile, Tmax_base, 1.10*l_cap_base, dt, trials);

S_vp   = (([vp_p10_s, vp_p10_p, vp_p10_v] - base_vec) ./ base_vec) / 0.10;
S_Rp   = (([Rp_p10_s, Rp_p10_p, Rp_p10_v] - base_vec) ./ base_vec) / 0.10;
S_Tmax = (([Tm_p10_s, Tm_p10_p, Tm_p10_v] - base_vec) ./ base_vec) / 0.10;
S_lcap = (([lc_p10_s, lc_p10_p, lc_p10_v] - base_vec) ./ base_vec) / 0.10;

%% ========================================================================
%% PRINT FORMATTED TABLES TO COMMAND WINDOW
%% ========================================================================
fprintf('=========================================================================\n');
fprintf('TABLE 1: SENSITIVITY TO PREDATOR SPEED v_p (v_e = 50 km/h, l_cap = 1.0 m)\n');
fprintf('=========================================================================\n');
fprintf('%-12s | %-12s | %-15s | %-15s | %-15s\n', ...
    'v_p (km/h)', 'Speed Ratio', 'Solitary (%)', 'Pair Pure (%)', 'Pair Voronoi (%)');
fprintf('-------------------------------------------------------------------------\n');
for i = 1:length(vp_kmh_vec)
    fprintf('%-12.1f | %-12.3f | %-15.1f | %-15.1f | %-15.1f\n', ...
        vp_kmh_vec(i), vp_kmh_vec(i)/50, rates_vp(i,1), rates_vp(i,2), rates_vp(i,3));
end
fprintf('\n');

fprintf('=========================================================================\n');
fprintf('TABLE 2: SENSITIVITY TO PREDATOR TURNING RADIUS R_p (r_jink in [2, 3] m)\n');
fprintf('=========================================================================\n');
fprintf('%-12s | %-12s | %-15s | %-15s | %-15s\n', ...
    'R_p (m)', 'w_p (rad/s)', 'Solitary (%)', 'Pair Pure (%)', 'Pair Voronoi (%)');
fprintf('-------------------------------------------------------------------------\n');
for i = 1:length(Rp_vec)
    w_curr = vp_base / Rp_vec(i);
    fprintf('%-12.2f | %-12.2f | %-15.1f | %-15.1f | %-15.1f\n', ...
        Rp_vec(i), w_curr, rates_Rp(i,1), rates_Rp(i,2), rates_Rp(i,3));
end
fprintf('\n');

fprintf('=========================================================================\n');
fprintf('TABLE 3: SENSITIVITY TO ANAEROBIC STAMINA T_max (l_cap = 1.0 m)\n');
fprintf('=========================================================================\n');
fprintf('%-12s | %-12s | %-15s | %-15s | %-15s\n', ...
    'T_max (s)', 'r_crit (m)', 'Solitary (%)', 'Pair Pure (%)', 'Pair Voronoi (%)');
fprintf('-------------------------------------------------------------------------\n');
for i = 1:length(Tmax_vec)
    rcrit = (vp_base - ve_base) * Tmax_vec(i) + l_cap_base;
    fprintf('%-12.1f | %-12.1f | %-15.1f | %-15.1f | %-15.1f\n', ...
        Tmax_vec(i), rcrit, rates_Tmax(i,1), rates_Tmax(i,2), rates_Tmax(i,3));
end
fprintf('\n');

fprintf('=========================================================================\n');
fprintf('TABLE 4: NORMALIZED SENSITIVITY ELASTICITY INDEX S_p (+10%% PERTURBATION)\n');
fprintf('=========================================================================\n');
fprintf('%-22s | %-15s | %-15s | %-15s\n', ...
    'Parameter (+10%)', 'Solitary S_p', 'Pair Pure S_p', 'Pair Voronoi S_p');
fprintf('-------------------------------------------------------------------------\n');
fprintf('%-22s | %+15.3f | %+15.3f | %+15.3f\n', 'Predator Speed (v_p)', S_vp(1), S_vp(2), S_vp(3));
fprintf('%-22s | %+15.3f | %+15.3f | %+15.3f\n', 'Stamina Limit (T_max)', S_Tmax(1), S_Tmax(2), S_Tmax(3));
fprintf('%-22s | %+15.3f | %+15.3f | %+15.3f\n', 'Turning Radius (R_p)', S_Rp(1), S_Rp(2), S_Rp(3));
fprintf('%-22s | %+15.3f | %+15.3f | %+15.3f\n', 'Capture Radius (l_cap)', S_lcap(1), S_lcap(2), S_lcap(3));
fprintf('=========================================================================\n');

%% ========================================================================
%% PLOT 3-PANEL SENSITIVITY FIGURE FOR LATEX REPORT
%% ========================================================================
figure('Color', 'w', 'Position', [100, 100, 1200, 380]);

subplot(1, 3, 1);
plot(vp_kmh_vec, rates_vp(:,1), '-o', 'LineWidth', 2, 'MarkerSize', 6); hold on;
plot(vp_kmh_vec, rates_vp(:,2), '-s', 'LineWidth', 2, 'MarkerSize', 6);
plot(vp_kmh_vec, rates_vp(:,3), '-^', 'LineWidth', 2, 'MarkerSize', 6);
grid on; xlabel('Predator Speed v_p (km/h)'); ylabel('Capture Rate (%)');
title('(a) Sensitivity to Predator Speed v_p');
legend('Solitary', 'Pair Pure Pursuit', 'Pair Voronoi', 'Location', 'southeast');
ylim([0 105]);

subplot(1, 3, 2);
plot(Rp_vec, rates_Rp(:,1), '-o', 'LineWidth', 2, 'MarkerSize', 6); hold on;
plot(Rp_vec, rates_Rp(:,2), '-s', 'LineWidth', 2, 'MarkerSize', 6);
plot(Rp_vec, rates_Rp(:,3), '-^', 'LineWidth', 2, 'MarkerSize', 6);
grid on; xlabel('Predator Turning Radius R_p (m)'); ylabel('Capture Rate (%)');
title('(b) Sensitivity to Turning Radius R_p');
legend('Solitary', 'Pair Pure Pursuit', 'Pair Voronoi', 'Location', 'southwest');
ylim([0 105]);

subplot(1, 3, 3);
plot(Tmax_vec, rates_Tmax(:,1), '-o', 'LineWidth', 2, 'MarkerSize', 6); hold on;
plot(Tmax_vec, rates_Tmax(:,2), '-s', 'LineWidth', 2, 'MarkerSize', 6);
plot(Tmax_vec, rates_Tmax(:,3), '-^', 'LineWidth', 2, 'MarkerSize', 6);
grid on; xlabel('Stamina Cutoff T_{max} (s)'); ylabel('Capture Rate (%)');
title('(c) Sensitivity to Stamina Limit T_{max}');
legend('Solitary', 'Pair Pure Pursuit', 'Pair Voronoi', 'Location', 'southeast');
ylim([0 105]);

sgtitle('Parameter Sensitivity Analysis Across Hunting Regimes (l_{cap} = 1.0 m)', 'FontWeight', 'bold');
saveas(gcf, 'sensitivity_analysis_plots.png');

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================
function [rate_sol, rate_pair, rate_vor] = run_all_strategies(v_p, v_e, R_p, R_e_sol, R_e_pair, T_max, l_cap, dt, trials)
    num_trials = length(trials.r_det);
    N = round(T_max / dt);
    jink_angle = deg2rad(110);
    
    w_p = v_p / R_p;
    w_e_sol  = v_e / R_e_sol;
    w_e_pair = v_e / R_e_pair;
    
    cap_sol = 0; cap_pair = 0; cap_vor = 0;
    
    for tr = 1:num_trials
        r_det  = trials.r_det(tr);
        a_sol  = trials.a_sol(tr);
        a_pair = trials.a_pair(tr);
        r_jink = trials.r_jink(tr);
        
        % --- 1. Solitary Pure Pursuit ---
        x_E = 0; y_E = 0; th_E = 0;
        x_P = -r_det * cos(a_sol); y_P = r_det * sin(a_sol); th_P = 0;
        is_jink = false; jink_dir = 1;
        for k = 1:N
            dist_P = hypot(x_E - x_P, y_E - y_P);
            if dist_P <= l_cap, cap_sol = cap_sol + 1; break; end
            th_LOS = atan2(y_E - y_P, x_E - x_P);
            dth_P = atan2(sin(th_LOS - th_P), cos(th_LOS - th_P));
            w_P_cmd = sign(dth_P) * min(abs(dth_P)/dt, w_p);
            
            % Hysteresis jink trigger in [2.0, 3.0] m
            if is_jink && dist_P > r_jink + 1.5
                is_jink = false;
            end
            if ~is_jink && dist_P <= r_jink
                is_jink = true;
                % Break orthogonal/away from current predator heading error
                jink_dir = sign(dth_P);
                if jink_dir == 0, jink_dir = 1; end
            end
            
            if is_jink
                % Continuously maintain 110-deg break angle relative to instantaneous LOS
                th_targ = th_LOS + jink_dir * jink_angle;
                dth_E = atan2(sin(th_targ - th_E), cos(th_targ - th_E));
                w_E_cmd = sign(dth_E) * min(abs(dth_E)/dt, w_e_sol);
            else
                % Run straight along current escape heading (as in Strategy 1)
                w_E_cmd = 0;
            end
            x_P = x_P + v_p*cos(th_P)*dt; y_P = y_P + v_p*sin(th_P)*dt; th_P = th_P + w_P_cmd*dt;
            x_E = x_E + v_e*cos(th_E)*dt; y_E = y_E + v_e*sin(th_E)*dt; th_E = th_E + w_E_cmd*dt;
        end
        
        % --- 2. Pair Pure Pursuit ---
        x_E = 0; y_E = 0; th_E = 0;
        x_P1 = -r_det*cos(a_pair); y_P1 =  r_det*sin(a_pair); th_P1 = 0;
        x_P2 = -r_det*cos(a_pair); y_P2 = -r_det*sin(a_pair); th_P2 = 0;
        is_jink = false;
        for k = 1:N
            d1 = hypot(x_E - x_P1, y_E - y_P1);
            d2 = hypot(x_E - x_P2, y_E - y_P2);
            min_d = min(d1, d2);
            if min_d <= l_cap, cap_pair = cap_pair + 1; break; end
            th_LOS1 = atan2(y_E - y_P1, x_E - x_P1);
            th_LOS2 = atan2(y_E - y_P2, x_E - x_P2);
            dth_P1 = atan2(sin(th_LOS1 - th_P1), cos(th_LOS1 - th_P1));
            dth_P2 = atan2(sin(th_LOS2 - th_P2), cos(th_LOS2 - th_P2));
            w_P1_cmd = sign(dth_P1) * min(abs(dth_P1)/dt, w_p);
            w_P2_cmd = sign(dth_P2) * min(abs(dth_P2)/dt, w_p);
            
            if is_jink && min_d > r_jink + 1.5
                is_jink = false;
            end
            if ~is_jink && min_d <= r_jink
                is_jink = true;
            end
            
            if is_jink
                if d1 <= d2
                    th_targ = th_LOS1 - jink_angle;
                else
                    th_targ = th_LOS2 + jink_angle;
                end
                dth_E = atan2(sin(th_targ - th_E), cos(th_targ - th_E));
            else
                vx_f = (x_E - x_P1)/d1 + (x_E - x_P2)/d2;
                vy_f = (y_E - y_P1)/d1 + (y_E - y_P2)/d2;
                th_f = atan2(vy_f, vx_f);
                dth_E = atan2(sin(th_f - th_E), cos(th_f - th_E));
            end
            w_E_cmd = sign(dth_E) * min(abs(dth_E)/dt, w_e_pair);
            x_P1 = x_P1 + v_p*cos(th_P1)*dt; y_P1 = y_P1 + v_p*sin(th_P1)*dt; th_P1 = th_P1 + w_P1_cmd*dt;
            x_P2 = x_P2 + v_p*cos(th_P2)*dt; y_P2 = y_P2 + v_p*sin(th_P2)*dt; th_P2 = th_P2 + w_P2_cmd*dt;
            x_E  = x_E  + v_e*cos(th_E)*dt;  y_E  = y_E  + v_e*sin(th_E)*dt;  th_E  = th_E  + w_E_cmd*dt;
        end
        
        % --- 3. Pair Bounded Voronoi (OA-ECBVC) ---
        x_E = 0; y_E = 0; th_E = 0;
        x_P1 = -r_det*cos(a_pair); y_P1 =  r_det*sin(a_pair); th_P1 = 0;
        x_P2 = -r_det*cos(a_pair); y_P2 = -r_det*sin(a_pair); th_P2 = 0;
        buffer = 1.5;
        Dx = max(abs([x_P1 - x_E, x_P2 - x_E])) + buffer;
        Dy = max(abs([y_P1 - y_E, y_P2 - y_E])) + buffer;
        for k = 1:N
            u_E = [x_E; y_E]; u_P1 = [x_P1; y_P1]; u_P2 = [x_P2; y_P2];
            d1 = hypot(x_E - x_P1, y_E - y_P1);
            d2 = hypot(x_E - x_P2, y_E - y_P2);
            if min(d1, d2) <= l_cap, cap_vor = cap_vor + 1; break; end
            buffer = max(0.05, buffer - 0.5 * dt);
            min_sx = max(abs([x_P1 - x_E, x_P2 - x_E])) + buffer;
            min_sy = max(abs([y_P1 - y_E, y_P2 - y_E])) + buffer;
            Dx = max(min_sx, Dx - v_p * dt);
            Dy = max(min_sy, Dy - v_p * dt);
            box_poly = [x_E-Dx, y_E-Dy; x_E+Dx, y_E-Dy; x_E+Dx, y_E+Dy; x_E-Dx, y_E+Dy];
            V_E  = clip_halfplane(clip_halfplane(box_poly, u_E,  u_P1), u_E,  u_P2);
            V_P1 = clip_halfplane(clip_halfplane(box_poly, u_P1, u_E),  u_P1, u_P2);
            V_P2 = clip_halfplane(clip_halfplane(box_poly, u_P2, u_E),  u_P2, u_P1);
            C_E  = poly_centroid(V_E,  u_E);
            C_P1 = poly_centroid(V_P1, u_P1);
            C_P2 = poly_centroid(V_P2, u_P2);
            targ_P1 = 0.35 * C_P1 + 0.65 * u_E;
            targ_P2 = 0.35 * C_P2 + 0.65 * u_E;
            th_vP1 = atan2(targ_P1(2) - y_P1, targ_P1(1) - x_P1);
            th_vP2 = atan2(targ_P2(2) - y_P2, targ_P2(1) - x_P2);
            th_vE  = atan2(C_E(2) - y_E, C_E(1) - x_E);
            dth_P1 = atan2(sin(th_vP1 - th_P1), cos(th_vP1 - th_P1));
            dth_P2 = atan2(sin(th_vP2 - th_P2), cos(th_vP2 - th_P2));
            dth_E  = atan2(sin(th_vE  - th_E),  cos(th_vE  - th_E));
            w_P1_cmd = sign(dth_P1) * min(abs(dth_P1)/dt, w_p);
            w_P2_cmd = sign(dth_P2) * min(abs(dth_P2)/dt, w_p);
            w_E_cmd  = sign(dth_E)  * min(abs(dth_E)/dt,  w_e_pair);
            x_P1 = x_P1 + v_p*cos(th_P1)*dt; y_P1 = y_P1 + v_p*sin(th_P1)*dt; th_P1 = th_P1 + w_P1_cmd*dt;
            x_P2 = x_P2 + v_p*cos(th_P2)*dt; y_P2 = y_P2 + v_p*sin(th_P2)*dt; th_P2 = th_P2 + w_P2_cmd*dt;
            x_E  = x_E  + v_e*cos(th_E)*dt;  y_E  = y_E  + v_e*sin(th_E)*dt;  th_E  = th_E  + w_E_cmd*dt;
        end
    end
    rate_sol  = (cap_sol  / num_trials) * 100;
    rate_pair = (cap_pair / num_trials) * 100;
    rate_vor  = (cap_vor  / num_trials) * 100;
end

function poly_out = clip_halfplane(poly_in, p_keep, p_other)
    if isempty(poly_in), poly_out = []; return; end
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
        t = max(0, min(1, (b - dot(a, p1)) / denom));
        pt = p1 + t * d;
    end
end

function C = poly_centroid(V, fallback_pos)
    if size(V, 1) < 3, C = fallback_pos; return; end
    x = V(:, 1); y = V(:, 2);
    x_next = [x(2:end); x(1)]; y_next = [y(2:end); y(1)];
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
