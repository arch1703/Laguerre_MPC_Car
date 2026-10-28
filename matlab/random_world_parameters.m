function config = random_world_parameters(overrides)
if nargin < 1, overrides = struct(); end
config = struct('mean_interval',6,'max_obstacles',6,'radius_min',0.08, ...
    'radius_max',0.14,'lateral_spread',0.10,'sensor_range',2.5, ...
    'sensor_fov',2*pi,'lifetime_min',Inf,'lifetime_max',Inf);
names = fieldnames(overrides);
for i = 1:numel(names)
    if ~isfield(config,names{i})
        error('mpc:UnknownWorldParameter','Unknown world parameter: %s',names{i});
    end
    config.(names{i}) = overrides.(names{i});
end
for name = {'mean_interval','radius_min','radius_max','sensor_range','sensor_fov'}
    validateattributes(config.(name{1}),{'numeric'}, ...
        {'scalar','real','finite','positive'},mfilename,name{1});
end
validateattributes(config.max_obstacles,{'numeric'},{'scalar','integer','nonnegative','finite'});
validateattributes(config.lateral_spread,{'numeric'},{'scalar','real','finite','nonnegative'});
for name = {'lifetime_min','lifetime_max'}
    validateattributes(config.(name{1}),{'numeric'}, ...
        {'scalar','real','positive'},mfilename,name{1});
end
if config.lifetime_max<config.lifetime_min || ...
        isinf(config.lifetime_min)~=isinf(config.lifetime_max)
    error('mpc:InvalidWorldParameters', ...
        'Lifetimes must be finite ordered bounds, or both Inf for permanent obstacles.');
end
if config.radius_max<config.radius_min || config.sensor_fov>2*pi
    error('mpc:InvalidWorldParameters','Require radius_max >= radius_min and sensor_fov <= 2*pi.');
end
end
