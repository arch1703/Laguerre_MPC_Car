function c = boundary_experiment_parameters(overrides)
if nargin<1, overrides = struct(); end
c = struct('seed',41,'line_noise',0,'obstacle_noise',0,'latency_steps',0, ...
    'line_loss_start',Inf,'line_loss_end',Inf,'miss_probability',0, ...
    'uncertain_classification',false,'max_line_age',0.4,'max_obstacle_age',0.4, ...
    'absence_confirmations',3,'replan_interval',0.15,'stall_timeout',12, ...
    'minimum_progress_ratio',0.35,'plant_wheelbase_scale',1, ...
    'plant_acceleration_scale',1,'start_fraction',0,'crossing_scale',1, ...
    'crossing_direction',1,'crossing_phase',0,'radius_scale',1, ...
    'model_margin',0,'enforce_deadline',false);
names = fieldnames(overrides);
for i = 1:numel(names)
    if ~isfield(c,names{i})
        error('mpc:UnknownExperimentParameter','Unknown experiment setting: %s',names{i});
    end
    c.(names{i}) = overrides.(names{i});
end
for name = {'line_noise','obstacle_noise','replan_interval','model_margin'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','nonnegative'});
end
for name = {'max_line_age','max_obstacle_age','stall_timeout', ...
        'plant_wheelbase_scale','plant_acceleration_scale','crossing_scale','radius_scale'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','finite','positive'});
end
validateattributes(c.seed,{'numeric'},{'scalar','integer','>=',0,'<=',2^32-1});
validateattributes(c.latency_steps,{'numeric'},{'scalar','integer','nonnegative'});
validateattributes(c.absence_confirmations,{'numeric'},{'scalar','integer','positive'});
validateattributes(c.miss_probability,{'numeric'},{'scalar','finite','>=',0,'<=',1});
validateattributes(c.start_fraction,{'numeric'},{'scalar','finite','>=',0,'<',1});
validateattributes(c.minimum_progress_ratio,{'numeric'},{'scalar','finite','>=',0,'<=',1});
validateattributes(c.crossing_direction,{'numeric'},{'scalar','integer','>=',-1,'<=',1,'nonzero'});
validateattributes(c.crossing_phase,{'numeric'},{'scalar','finite'});
validateattributes(c.uncertain_classification,{'logical'},{'scalar'});
validateattributes(c.enforce_deadline,{'logical'},{'scalar'});
for name = {'line_loss_start','line_loss_end'}
    validateattributes(c.(name{1}),{'numeric'},{'scalar','real','nonnegative','nonnan'});
end
if c.line_loss_end<c.line_loss_start
    error('mpc:InvalidExperiment','Line-loss end must not precede its start.');
end
end
