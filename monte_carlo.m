% solitary_capture_rate_analysis.m
% Monte Carlo simulation evaluating average capture rate with 
% biomechanically matched 1.26x G-force ratio parameters.
clear; clc;

%% 1. Biomechanical Parameter Setup
v_p = 60 / 3.6;          % Pursuer speed (16.67 m/s)
v_e = 50 / 3.6;          % Evader speed (13.89 m/s)

% Tuning Turning Radii to achieve ~25% higher G-force for the predator
R_p = 4.0;               % Pursuer min turning radius (m)
R_e = 3.5;               % Evader min turning radius (m)
l_cap = 0.5;             % Center-mass capture radius (m)

a_cp = (v_p^2) / R_p;    % Predator centripetal accel (69.44 m/s^2)
a_ce = (v_e^2) / R_e;    % Prey centripetal accel (55.11 m/s^2)
g_p = a_cp / 9.81;       % 7.08g
g_e = a_ce / 9.81;       % 5.62g

fprintf('--- Biomechanical Validation ---\n');
fprintf('Predator max G-force: %.2fg\n', g_p);
fprintf('Prey max G-force: %.2fg\n', g_e);
fprintf('Predator/Prey G-Force Ratio: %.2fx\n\n', g_p / g_e);

%% 2. Monte Carlo Simulation Setup
num_trials = 1000;
T_max = 15.0;            % Raptor stamina limit (s)
dt = 0.01;               % Time step (s)
N = round(T_max / dt);

w_p_max = v_p / R_p;
w_e_max = v_e / R_e;
jink_angle = deg2rad(110); % Optimal evasion angle

% Pre-generate randomized encounter conditions
% 1. Detection distance: Uniformly distributed between 15m and 50m
r_det_array = 15 + (50 - 15) * rand(1, num_trials);
% 2. Attack angle: Uniformly distributed between -45 and 45 degrees
alpha_array = deg2rad(-45 + 90 * rand(1, num_trials));
% 3. Prey Jink Trigger: Uniformly distributed between 2.0m and 5.0m
r_jink_array = 2.0 + (5.0 - 2.0) * rand(1, num_trials);

captures = 0;

%% 3. Simulation Loop
fprintf('Running %d solitary pursuit trials...\n', num_trials);
for trial = 1:num_trials
    r_det = r_det_array(trial);
    alpha = alpha_array(trial);
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
            captures = captures + 1;
            break;
        end
        
        % Pursuer Control
        theta_LOS = atan2(y_E - y_P, x_E - x_P);
        dtheta_P = atan2(sin(theta_LOS - theta_P), cos(theta_LOS - theta_P));
        w_P = sign(dtheta_P) * min(abs(dtheta_P)/dt, w_p_max);
        
        % Evader Control (With 4.0m reset buffer)
        if is_jinking && dist_P > r_jink + 4.0
            is_jinking = false;
        end
        
        if ~is_jinking && dist_P <= r_jink
            is_jinking = true;
            % Prey breaks away from the line-of-sight threat
            theta_target = theta_LOS + jink_angle; 
        end
        
        if is_jinking
            dtheta_E = atan2(sin(theta_target - theta_E), cos(theta_target - theta_E));
        else
            % Straight line fleeing from the threat
            dtheta_E = atan2(sin(theta_LOS - theta_E), cos(theta_LOS - theta_E));
        end
        w_E = sign(dtheta_E) * min(abs(dtheta_E)/dt, w_e_max);
        
        % Kinematic Update
        x_P = x_P + v_p * cos(theta_P) * dt;
        y_P = y_P + v_p * sin(theta_P) * dt;
        theta_P = theta_P + w_P * dt;
        
        x_E = x_E + v_e * cos(theta_E) * dt;
        y_E = y_E + v_e * sin(theta_E) * dt;
        theta_E = theta_E + w_E * dt;
    end
end

capture_rate = (captures / num_trials) * 100;
fprintf('--- Simulation Results ---\n');
fprintf('Total Captures: %d / %d\n', captures, num_trials);
fprintf('Average Solitary Capture Rate: %.1f%%\n', capture_rate);

% Empirical comparison
fprintf('\n--- Empirical Comparison ---\n');
fprintf('Simulated Velociraptor Rate: %.1f%%\n', capture_rate);
fprintf('Empirical Lion (Solitary):   15%% - 25%%\n');
fprintf('Empirical Cheetah (Solitary): 50%% - 58%%\n');