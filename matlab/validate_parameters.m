function p = validate_parameters(p)
positive = {'L','car_front','car_rear','car_half_width','T','v_max','v_tgt', ...
    'delta_max','a_max','delta_rate_max','jerk_max','clearance', ...
    'road_half_width','lateral_accel_max','detour_offset','detour_length', ...
    'obstacle_radius','track_a','track_b','solver_tolerance'};
for i = 1:numel(positive)
    validateattributes(p.(positive{i}),{'numeric'}, ...
        {'scalar','real','finite','positive'},mfilename,positive{i});
end
validateattributes(p.Np,{'numeric'},{'scalar','integer','>=',2});
validateattributes(p.Nc,{'numeric'},{'scalar','integer','>=',1,'<=',p.Np});
validateattributes(p.N_wp,{'numeric'},{'scalar','integer','>=',20});
validateattributes(p.safety_substeps,{'numeric'},{'scalar','integer','positive'});
validateattributes(p.obstacle_count,{'numeric'},{'scalar','integer','nonnegative'});
validateattributes(p.laguerre_order,{'numeric'},{'scalar','integer','positive','<=',p.Nc});
validateattributes(p.laguerre_alpha,{'numeric'},{'scalar','>',0,'<',1});
validateattributes(p.Q_,{'numeric'},{'vector','numel',4,'finite','nonnegative'});
validateattributes(p.R_,{'numeric'},{'vector','numel',2,'finite','positive'});
validateattributes(p.a_min,{'numeric'},{'scalar','finite','<',0});
validateattributes(p.v_min,{'numeric'},{'scalar','finite','nonnegative','<',p.v_max});
validateattributes(p.v_tgt,{'numeric'},{'>=',p.v_min,'<=',p.v_max});
validateattributes(p.delta_max,{'numeric'},{'<',pi/2});
validateattributes(p.linearization_guard,{'logical'},{'scalar'});
for name = {'linearization_position_tolerance','linearization_heading_tolerance', ...
        'linearization_steering_trust','linearization_acceleration_trust'}
    validateattributes(p.(name{1}),{'numeric'},{'scalar','finite','positive'});
end
validateattributes(p.linearization_iterations,{'numeric'},{'scalar','integer','>=',1});
validateattributes(p.qp_max_iterations,{'numeric'},{'scalar','integer','positive'});
p.Q_ = p.Q_(:)';
p.R_ = p.R_(:)';
p.body_offset = (p.car_front-p.car_rear)/2;
p.body_radius = hypot((p.car_front+p.car_rear)/2,p.car_half_width);
if p.body_radius+p.clearance >= p.road_half_width
    error('mpc:InvalidRoad','The vehicle and clearance margin do not fit within the road.');
end
end
