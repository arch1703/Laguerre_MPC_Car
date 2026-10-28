function result = run_boundary_demo(kind,options)
arguments
    kind (1,:) char = 'chicane'
    options.Duration (1,1) double {mustBePositive,mustBeFinite} = 35
    options.Save (1,1) logical = true
    options.Plot (1,1) logical = false
    options.Boundaries (1,1) struct = struct()
    options.Controller (1,:) char {mustBeMember(options.Controller,{'laguerre','ltv'})} = 'laguerre'
    options.Experiment (1,1) struct = struct()
    options.OutputRoot (1,:) char = fullfile(fileparts(mfilename('fullpath')),'..','results','runs')
end
p = parameters();
p.Np = 12; p.Nc = 6; p.v_tgt = 0.8;
p.linearization_guard = true;
p = validate_parameters(p);
c = boundary_experiment_parameters(options.Experiment);
truth_track = make_boundary_track(kind,p,options.Boundaries);
world = CrossingObstacleWorld(truth_track,c);
sensor = BoundarySensor(c);
plant = p; plant.L = p.L*c.plant_wheelbase_scale;
station = c.start_fraction*truth_track.length;
grid = [truth_track.s;truth_track.length];
x = [interp1(grid,[truth_track.X;truth_track.X(1)],station); ...
    interp1(grid,[truth_track.Y;truth_track.Y(1)],station); ...
    interp1(grid,[truth_track.psi;truth_track.psi(1)+2*pi],station);0.8];
initial = x; previous = zeros(2,1); warm = repmat(previous,1,p.Nc);
N = round(options.Duration/p.T);
values = zeros(N,29); step_time = zeros(N,1);
known_ids = strings(N,1); statuses = strings(N,1);
called = false(N,1); valid_predictions = false(N,1);
contacts = false(N,1); yielding = false(N,1); applied = true(N,1);
world_counts = zeros(N,1); observed_gap = Inf(N,1);
model_errors = zeros(N,2); passes = zeros(N,1);
world_history = cell(N,1); line_history = cell(N,1);
measurement_history = cell(N,1);
stage_times = zeros(N,5); ages = zeros(N,2);
replanned = false(N,1); reserves = false(N,1); deadlines = false(N,1);
events = struct('time',{},'type',{},'detail',{});
plans = struct('time',{},'path',{});
termination = 'completed'; last_yielding = false;
brake = p;
brake.Np = 24;
brake.brake_terminal_acceleration = true;
cached_path = []; cached_obstacles = zeros(0,4); cached_available = false;
last_plan = -Inf; stalled_time = 0;
for k = 1:N
    start = tic; time = (k-1)*p.T;
    world.advance(time);
    [corridor,measured,lines,world_history{k},sensing] = ...
        sensor.observe(truth_track,world,x,time);
    line_history{k} = lines;
    measured.t = repmat(time,height(measured),1);
    measurement_history{k} = measured;
    % Keep full memory in the recording; exclusions require a visibility certificate.
    measured = measured(~measured.outside_visible,:);
    stage_times(k,1) = toc(start);
    ages(k,:) = [sensing.line_age,sensing.obstacle_age];
    obstacles = world.snapshot();
    world_counts(k) = height(obstacles);
    planning_start = tic;
    active_p = p;
    freshness = max(0.25,1-max(sensing.line_age/c.max_line_age, ...
        sensing.obstacle_age/c.max_obstacle_age));
    active_p.v_tgt = p.v_tgt*freshness;
    % A direction-free reachable disc, not a velocity estimate or route forecast.
    guarded = measured;
    guarded.R = measured.R+measured.position_bound+c.model_margin+ ...
        measured.speed_bound.*(time-measured.last_seen+p.Np*p.T+p.T);
    geometry = guarded{:,{'id','X','Y','R'}};
    changed = ~isequal(size(geometry),size(cached_obstacles));
    if ~changed && ~isempty(geometry)
        changed = any(geometry(:,1)~=cached_obstacles(:,1)) || ...
            max(abs(geometry(:,2:4)-cached_obstacles(:,2:4)),[],'all')>0.04;
    end
    if isempty(cached_path) || time-last_plan>=c.replan_interval-1e-9 || changed
        [cached_path,cached_available] = plan_sensed_path(corridor,guarded,x,active_p);
        cached_obstacles = geometry; last_plan = time; replanned(k) = true;
        plans(end+1) = struct('time',time,'path',cached_path); %#ok<AGROW>
    end
    path = cached_path; path.v = min(path.v,active_p.v_tgt);
    available = cached_available && ~sensing.line_stale && ~sensing.obstacle_stale;
    station = project_path(corridor,x(1:2));
    for j = 1:height(measured)
        if measured.speed_bound(j)==0, continue; end
        [s,lateral] = project_path(corridor,[measured.X(j),measured.Y(j)]);
        stopping_time = brake.Np*p.T+p.T;
        risk_width = max(max(corridor.half_width)+measured.R(j)+p.clearance, ...
            guarded.R(j)+p.body_radius+p.clearance+measured.speed_bound(j)*stopping_time);
        yield_distance = p.v_tgt*stopping_time+p.car_front+measured.R(j)+ ...
            measured.speed_bound(j)*stopping_time+p.clearance;
        if s>station-0.20 && s-station<yield_distance && lateral<risk_width
            available = false;
        end
    end
    if last_yielding && (x(4)>1e-4 || abs(previous(2))>1e-3)
        available = false;
    end
    if ~available, path.v(:) = 0; end
    [reference,current] = make_reference(path,x,p);
    planning_time = toc(planning_start);
    solve_start = tic;
    flag = 1; iterations = 0; variables = 0; violation = 0;
    fallback = false; valid = false;
    if available
        called(k) = true;
        [sequence,diagnostic] = mpc_solve(x,previous,reference,corridor,guarded,p,options.Controller,warm);
        flag = diagnostic.exitflag; iterations = diagnostic.iterations;
        variables = diagnostic.variables; violation = diagnostic.constraint_violation;
        model_errors(k,:) = [diagnostic.model_position_error,diagnostic.model_heading_error];
        passes(k) = diagnostic.linearization_passes;
    end
    solve_time = toc(solve_start);
    validation_start = tic;
    braking_obstacles = measured;
    braking_obstacles.R = measured.R+measured.position_bound+c.model_margin+ ...
        measured.speed_bound.*(time-measured.last_seen+brake.Np*p.T+2*p.T);
    if available
        if flag>0
            [valid,~] = check_control_sequence(x,previous,sequence,guarded,corridor,p);
            valid = valid && check_sensor_visibility(x,sequence,p,sensing);
            valid_predictions(k) = valid;
            if valid
                next = bicycle_step(x,sequence(:,1),p);
                [reserve_sequence,reserve] = braking_sequence(next,sequence(:,1),braking_obstacles,corridor,brake);
                reserve = reserve && check_sensor_visibility(next,reserve_sequence,brake,sensing,p.T);
                reserves(k) = reserve;
                valid = valid && reserve;
            end
        end
        fallback = ~valid_predictions(k) || flag<=0;
    end
    deadlines(k) = toc(start)>p.T;
    if c.enforce_deadline && deadlines(k), valid = false; fallback = available; end
    if ~available || ~valid
        yielding(k) = true;
        [sequence,valid] = braking_sequence(x,previous,braking_obstacles,corridor,brake);
        valid = valid && check_sensor_visibility(x,sequence,brake,sensing);
        reserves(k) = valid;
        statuses(k) = "yielding";
        if x(4)<1e-4, statuses(k) = "waiting"; end
        if fallback
            events(end+1) = struct('time',time,'type','controller_rejection', ...
                'detail',sprintf('flag=%d; nonlinear validation rejected=%d',flag,~valid_predictions(k))); %#ok<AGROW>
            warning('mpc:BoundaryFallback','Rejected MPC action at %.2f s; attempting checked braking.',time);
        end
    else
        statuses(k) = "tracking_lines";
        if ~isempty(measured), statuses(k) = "avoiding"; end
    end
    if yielding(k)~=last_yielding
        label = 'resume';
        if yielding(k), label = 'yield'; end
        events(end+1) = struct('time',time,'type',label, ...
            'detail','Changed between verified tracking and conservative yielding'); %#ok<AGROW>
        last_yielding = yielding(k);
    end
    stop = ~valid;
    u = sequence(:,1);
    if stop
        termination = 'no_verified_control';
        statuses(k) = termination;
        applied(k) = false;
        warning('mpc:NoSafeControl','Neither tracking nor bounded-motion braking was verified at %.2f s.',time);
    end
    [~,observed_gap(k)] = check_motion(x,u,measured,corridor,p);
    stage_times(k,2) = toc(validation_start);
    stage_times(k,3) = toc(start);
    monitor_start = tic;
    plant_u = [u(1);u(2)*c.plant_acceleration_scale];
    [gap,edge,rectangle,contact] = check_moving_motion(x,plant_u,world,time,truth_track,plant);
    if stop
        [gap,edge,rectangle] = vehicle_clearance(x,obstacles,truth_track,p);
        contact = rectangle<=0;
    end
    contacts(k) = contact;
    stage_times(k,4) = toc(monitor_start);
    validation_time = stage_times(k,2);
    [~,cte] = project_path(truth_track,x(1:2));
    [~,path_error] = project_path(path,x(1:2));
    distance = NaN; active = 0;
    if ~isempty(measured)
        [distance,index] = min(hypot(measured.X-x(1),measured.Y-x(2)));
        active = measured.id(index);
    end
    known_ids(k) = string(jsonencode(measured.id'));
    values(k,:) = [time,x',u',current,reference(1,1:2),cte,path_error, ...
        solve_time,flag,iterations,variables,fallback,gap,edge,rectangle, ...
        height(measured),active,distance,violation,validation_time,planning_time];
    if ~stop
        x = bicycle_step(x,plant_u,plant); previous = u;
        warm = repmat(u,1,p.Nc);
        if ~yielding(k), warm = [sequence(:,2:p.Nc),sequence(:,p.Nc)]; end
    end
    step_time(k) = toc(start);
    if x(4)<0.05, stalled_time = stalled_time+p.T;
    else, stalled_time = 0;
    end
    stage_times(k,5) = stalled_time;
    if contact
        termination = 'collision';
        statuses(k) = termination;
        warning('mpc:Collision','Physical monitor confirmed contact at %.2f s.',time);
        break
    end
    if stop, break; end
    if stalled_time>=c.stall_timeout
        termination = 'stalled';
        statuses(k) = termination;
        warning('mpc:Stalled','Vehicle made no meaningful progress for %.1f seconds.',stalled_time);
        break
    end
end
names = {'t','X','Y','psi','v','delta','a','X_ref','Y_ref','psi_ref','v_ref', ...
    'X_target','Y_target','track_cte','path_error','solve_time','exitflag', ...
    'iterations','variables','fallback','min_clearance','road_clearance', ...
    'rectangle_clearance','visible_obstacles','active_obstacle','min_obs_dist', ...
    'constraint_violation','validation_time','planning_time'};
logs = array2table(values(1:k,:),'VariableNames',names);
logs.status = statuses(1:k); logs.applied = applied(1:k);
logs.step_time = step_time(1:k); logs.solver_called = called(1:k);
logs.prediction_valid = valid_predictions(1:k); logs.confirmed_contact = contacts(1:k);
logs.world_obstacles = world_counts(1:k); logs.known_obstacle_ids = known_ids(1:k);
logs.observed_clearance = observed_gap(1:k); logs.yielding = yielding(1:k);
logs.model_position_error = model_errors(1:k,1);
logs.model_heading_error = model_errors(1:k,2);
logs.linearization_passes = passes(1:k);
logs.sensor_time = stage_times(1:k,1);
logs.online_time = stage_times(1:k,3);
logs.physics_monitor_time = stage_times(1:k,4);
logs.stopped_duration = stage_times(1:k,5);
logs.line_age = ages(1:k,1); logs.obstacle_age = ages(1:k,2);
logs.replanned = replanned(1:k); logs.stopping_reserve = reserves(1:k);
logs.deadline_intervention = deadlines(1:k) & c.enforce_deadline;
metrics = evaluate_run(logs,x,truth_track,p);
metrics.termination = termination;
metrics.success = metrics.success && strcmp(termination,'completed') && ~any(logs.fallback);
metrics.yielding_steps = sum(logs.yielding);
metrics.resume_events = sum(strcmp({events.type},'resume'));
metrics.required_laps = c.minimum_progress_ratio*p.v_tgt*N*p.T/truth_track.length;
metrics.progress_sufficient = metrics.laps>=metrics.required_laps;
metrics.safety_pass = metrics.min_clearance>=p.clearance-p.solver_tolerance && ...
    metrics.min_road_clearance>=p.clearance-p.solver_tolerance && ~any(contacts(1:k));
metrics.success = metrics.success && metrics.progress_sufficient;
metrics.road_departure_steps = sum(logs.road_clearance<0);
metrics.max_stopped_duration = max(logs.stopped_duration);
metrics.online_time_p99 = prctile(logs.online_time,99);
metrics.missed_online_deadlines = sum(logs.online_time>p.T);
metrics.deadline_interventions = sum(logs.deadline_intervention);
metrics.replans = sum(logs.replanned);
for name = {'sensor_time','planning_time','solve_time','validation_time','physics_monitor_time'}
    metrics.([name{1},'_p99']) = prctile(logs.(name{1}),99);
end
world_metadata = world.metadata();
world_metadata.track_kind = kind;
world_metadata.experiment = c;
world_metadata.memory = 'bounded_measurement_memory_with_negative_confirmation';
result = struct('controller',options.Controller,'scenario','boundary_crossing', ...
    'parameters',p,'seed',c.seed,'track',truth_track,'path',path, ...
    'obstacles',world.snapshot(0),'logs',logs,'initial_state',initial,'final_state',x, ...
    'metrics',metrics,'events',events,'plans',plans,'world',world_metadata, ...
    'requested_duration',N*p.T,'run_dir','', ...
    'obstacle_history',vertcat(world_history{1:k}), ...
    'line_history',vertcat(line_history{1:k}), ...
    'measurement_history',vertcat(measurement_history{1:k}), ...
    'experiment',c);
% Include all realized IDs, even if an object was absent at the start or end.
history = result.obstacle_history;
[~,first] = unique(history.id,'stable');
result.obstacles = history(first,{'id','X','Y','R','spawn_time','removal_time'});
if options.Save, result.run_dir = save_mpc_run(result,options.OutputRoot); end
if options.Plot
    figure_handle = plot_results(result);
    if options.Save, exportgraphics(figure_handle,fullfile(result.run_dir,'summary.png')); end
end
fprintf('boundary/%s: %s, %.2f s, %.2f laps, %d yielding steps; %s\n', ...
    kind,termination,metrics.duration,metrics.laps,metrics.yielding_steps,result.run_dir);
end
