% monte_carlo_capture_rate_analysis.m
% Monte Carlo simulation evaluating and comparing average capture rates for:
%   Part 1: Solitary Hunting (1.26x G-force ratio, benchmarked against Lions/Cheetahs)
%   Part 2: Pair Hunting     (1.85x G-force ratio, benchmarked against Wolf Packs)
clear; clc;
rng(42); % Fixed seed for reproducible report statistics

%% 1. Shared Kinematic Parameters
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)
l_cap = 0.5;             % Center-mass capture radius (m)

num_trials = 1000;
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.01;               % Time step (s)
N = round(T_max / dt);
jink_angle = deg2rad(110); % Optimal break-turn angle

% Pre-generate randomized encounter conditions (shared across both regimes)
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
% Tuning Turning Radii to match the 1.85x Wolf-Elk allometric G-force ratio:
% (g_p / g_e) = (v_p^2 / R_p) / (v_e^2 / R_e) = (60/50)^2 * (R_e / R_p) = 1.44 * (R_e / R_p)
% Setting R_p = 4.0 m and R_e = 5.14 m yields 1.44 * (5.14 / 4.0) = 1.8504x
R_p_pair = 4.0;          % Pursuer min turning radius (m)
R_e_pair = 5.14;         % Evader min turning radius (m) for 1.85x ratio
w_p_pair = v_p / R_p_pair;
w_e_pair = v_e / R_e_pair;

g_p_pair = ((v_p^2) / R_p_pair) / 9.81; % 7.08g
g_e_pair = ((v_e^2) / R_e_pair) / 9.81; % 3.83g

fprintf('=========================================================\n');
fprintf('PART 2: PAIR PURSUIT BIOMECHANICAL VALIDATION (WOLF RATIO)\n');
fprintf('=========================================================\n');
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
            % Break away from whichever pursuer is closer (away from the pincer)
            if d1 <= d2
                theta_target = theta_LOS1 - jink_angle;
            else
                theta_target = theta_LOS2 + jink_angle;
            end
        end
        
        if is_jinking
            dtheta_E = atan2(sin(theta_target - theta_E), cos(theta_target - theta_E));
        else
            % Flee along the threat bisector away from both pursuers
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
fprintf('Pair Captures:     %d / %d (%.1f%%)\n\n', ...
    captures_pair, num_trials, rate_pair);

%% ========================================================================
%% 3. Summary & Empirical Comparison Output
%% ========================================================================
fprintf('=========================================================\n');
fprintf('SUMMARY: SIMULATED VS. EMPIRICAL CAPTURE RATES\n');
fprintf('=========================================================\n');
fprintf('Solitary Simulation (1.26x G-Ratio): %.1f%%\n', rate_solitary);
fprintf('  -> Empirical Lion (Solitary):      15%% - 25%%\n');
fprintf('  -> Empirical Cheetah (Solitary):   50%% - 58%%\n\n');
fprintf('Pair Simulation (1.85x Wolf Ratio):  %.1f%%\n', rate_pair);
fprintf('  -> Empirical Wolf Pack (vs. Elk):  ~20%%\n');
fprintf('  -> Empirical Lion Pride (Pack):    30%% - 40%%\n');
fprintf('=========================================================\n');
