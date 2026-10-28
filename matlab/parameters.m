function p = parameters()
% All physical and MPC tuning parameters for the RC toy car simulation.

% --- Vehicle (RC car scale) ---
p.m = 1.5;          % mass (kg)
p.L = 0.25;         % wheelbase (m)
p.car_front = 0.29; % distance from rear axle to front bumper (m)
p.car_rear = 0.06;
p.car_half_width = 0.075;
p.body_offset = (p.car_front - p.car_rear) / 2;
p.body_radius = hypot((p.car_front + p.car_rear) / 2, p.car_half_width);

% --- Hard constraints ---
p.delta_max =  deg2rad(22);   % max steering angle (rad)
p.a_max     =  6.67;          % max acceleration (m/s^2) = 2 * 5 N / 1.5 kg
p.a_min     = -4.00;          % max braking (m/s^2)
p.v_max     =  3.0;           % max speed (m/s)
p.v_min     =  0.0;           % no reversing
p.delta_rate_max = deg2rad(120); % rad/s
p.jerk_max = 12.0;              % m/s^3
p.clearance = 0.05;             % additional clearance outside vehicle body (m)
p.road_half_width = 0.85;
p.lateral_accel_max = 1.8;

% --- MPC ---
p.T     = 0.05;    % sample time (s)  -> 20 Hz
p.Np    = 20;      % prediction horizon (steps = 1.0 s)
p.Nc    = 10;      % control horizon   (steps = 0.5 s)
p.v_tgt = 1.2;     % target speed (m/s)

% --- Cost weights ---
% psi uses atan2-safe angle cost; Q_v holds speed near v_tgt for curve stability
p.Q_ = [30, 30, 2, 4];     % [X, Y, psi, v], shared by all controllers
p.R_ = [0.3, 0.05];         % [delta, a] input rate smoothness
p.laguerre_alpha = 0.70;
p.laguerre_order = 4;
p.detour_offset = 0.52;
p.detour_length = 1.8;      % half-length of smooth detour, measured along track
p.obstacle_radius = 0.12;
p.obstacle_count = 3;
p.solver_tolerance = 1e-6;
p.safety_substeps = 10;
p.linearization_guard = false;
p.linearization_position_tolerance = 0.015;
p.linearization_heading_tolerance = 0.03;
p.linearization_steering_trust = 0.12;
p.linearization_acceleration_trust = 1.0;
p.linearization_iterations = 6;
p.qp_max_iterations = 100;

% --- Oval track ---
p.track_a = 2.0;    % semi-major axis in X (m)
p.track_b = 1.5;    % semi-minor axis in Y (m); b=1 was too tight (rho_min=0.5m < R_min=0.62m)
p.N_wp    = 400;    % number of waypoints
end
