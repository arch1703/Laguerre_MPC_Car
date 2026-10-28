function result = run_mpc_demo(controller,scenario,options)
arguments
    controller (1,:) char {mustBeMember(controller,{'laguerre','ltv','nonlinear'})} = 'laguerre'
    scenario (1,:) char {mustBeMember(scenario,{'tracking','static','appearing','random','moving'})} = 'tracking'
    options.Duration (1,1) double {mustBePositive,mustBeFinite} = 15
    options.Save (1,1) logical = true
    options.Plot (1,1) logical = false
    options.Parameters (1,1) struct = struct()
    options.Seed (1,1) double {mustBeInteger,mustBeNonnegative} = 21
    options.OutputRoot (1,:) char = fullfile(fileparts(mfilename('fullpath')),'..','results','runs')
    options.Obstacles table = table()
    options.InitialState double = []
    options.WorldParameters (1,1) struct = struct()
    options.WorldSeed double = []
end
p = parameters();
names = fieldnames(options.Parameters);
for i = 1:numel(names)
    if ~isfield(p,names{i}) || ismember(names{i},{'body_offset','body_radius'})
        error('mpc:UnknownParameter','Unknown parameter: %s',names{i});
    end
    p.(names{i}) = options.Parameters.(names{i});
end
p = validate_parameters(p);
validateattributes(options.Duration/p.T,{'double'},{'>=',1});
if strcmp(scenario,'tracking') && height(options.Obstacles)>0
    error('mpc:InvalidScenario','Use static or appearing for a nonempty obstacle layout.');
end
track = generate_track(p);
independent_world = strcmp(scenario,'random');
moving_world = strcmp(scenario,'moving');
if ~independent_world && (~isempty(options.WorldSeed) || ~isempty(fieldnames(options.WorldParameters)))
    error('mpc:InvalidScenario','WorldSeed and WorldParameters apply only to random mode.');
end
if moving_world
    if width(options.Obstacles)>0
        error('mpc:InvalidScenario','Moving mode owns its independent world layout.');
    end
    world = MovingObstacleWorld(track);
    obstacles = world.snapshot();
elseif independent_world
    if width(options.Obstacles)>0
        error('mpc:InvalidScenario','Random mode does not accept a prewritten obstacle schedule.');
    end
    world = RandomObstacleWorld(track,options.WorldParameters,options.WorldSeed);
    obstacles = world.snapshot();
elseif width(options.Obstacles) > 0
    obstacles = options.Obstacles;
    required = {'X','Y','R','spawn_time'};
    if ~all(ismember(required,obstacles.Properties.VariableNames)) || ...
            any(~isfinite(obstacles{:,required}),'all') || ...
            any(obstacles.R<=0) || any(obstacles.spawn_time<0)
        error('mpc:InvalidObstacles','Obstacles need finite X,Y,R,spawn_time; R>0 and spawn_time>=0.');
    end
else
    obstacles = make_demo_obstacles(track,p,scenario,options.Duration,options.Seed);
end
visible = obstacles(obstacles.spawn_time<=0,:);
if moving_world, visible = obstacles([],:); end
path = plan_reference_path(track,visible,p);
x = [path.X(1);path.Y(1);path.psi(1);path.v(1)];
if ~isempty(options.InitialState)
    validateattributes(options.InitialState,{'double'},{'vector','numel',4,'finite'});
    x = options.InitialState(:);
end
if x(4)<p.v_min || x(4)>p.v_max
    error('mpc:InvalidInitialState','Initial speed must satisfy the configured bounds.');
end
initial = x;
previous = [0;0];
warm = repmat(previous,1,p.Nc);
if strcmp(controller,'nonlinear')
    [model,holder,nloptions] = create_nonlinear_mpc(p);
end
N = round(options.Duration/p.T);
values = zeros(N,29);
status = strings(N,1);
step_times = zeros(N,1);
coefficients = NaN(N,2*p.laguerre_order);
solver_called = false(N,1);
prediction_valid = false(N,1);
world_counts = zeros(N,1);
new_detections = zeros(N,1);
new_removals = zeros(N,1);
removal_detections = zeros(N,1);
total_spawned = zeros(N,1);
total_removed = zeros(N,1);
observed_clearance = Inf(N,1);
known_ids = strings(N,1);
confirmed_contact = false(N,1);
termination = 'completed';
events = struct('time',{},'type',{},'detail',{});
plans = struct('time',0,'path',path);
braking = false;
deferred = false(height(obstacles),1);
if ~independent_world
    stream = RandStream('mt19937ar','Seed',options.Seed);
end
old_visible_count = height(visible);
old_visible_ids = zeros(0,1);
old_visible_geometry = zeros(0,3);
obstacle_history = cell(N,1);
for k = 1:N
    step_start = tic;
    time = (k-1)*p.T;
    if moving_world
        world.advance(time);
        [visible,obstacle_history{k}] = world.observe(x,time);
        obstacles = world.snapshot();
        physical_obstacles = obstacles;
        total_spawned(k) = height(obstacles);
    elseif independent_world
        [spawned,removed] = world.advance(time);
        [visible,discovered,cleared] = world.observe(x,time);
        obstacles = world.snapshot();
        physical_obstacles = obstacles(isnan(obstacles.removal_time),:);
        new_detections(k) = height(discovered);
        new_removals(k) = height(removed);
        removal_detections(k) = height(cleared);
        total_spawned(k) = height(obstacles);
        total_removed(k) = sum(isfinite(obstacles.removal_time));
        for j = 1:height(spawned)
            events(end+1) = struct('time',time,'type','world_spawn', ...
                'detail',sprintf('Obstacle %d at (%.3f, %.3f), radius %.3f', ...
                spawned.id(j),spawned.X(j),spawned.Y(j),spawned.R(j))); %#ok<AGROW>
        end
        for j = 1:height(discovered)
            events(end+1) = struct('time',time,'type','detection', ...
                'detail',sprintf('Sensor discovered obstacle %d',discovered.id(j))); %#ok<AGROW>
        end
        for j = 1:height(removed)
            events(end+1) = struct('time',time,'type','world_remove', ...
                'detail',sprintf('Obstacle %d disappeared',removed.id(j))); %#ok<AGROW>
        end
        for j = 1:height(cleared)
            events(end+1) = struct('time',time,'type','removal_detection', ...
                'detail',sprintf('Sensor confirmed obstacle %d is absent',cleared.id(j))); %#ok<AGROW>
        end
    else
    new = find(obstacles.spawn_time<=time & ~isfinite(obstacles.X));
    for j = new'
        progress = project_path(track,x(1:2));
        ahead = p.detour_length+0.8+0.2*rand(stream);
        candidates = mod(progress+linspace(ahead,track.length-ahead,100),track.length);
        placed = find(isfinite(obstacles.X));
        placed_s = zeros(numel(placed),1);
        for index = 1:numel(placed)
            placed_s(index) = project_path(track, ...
                [obstacles.X(placed(index)),obstacles.Y(placed(index))]);
        end
        allowed = repmat(track.length>2*ahead,size(candidates));
        for position = placed_s'
            separation = abs(mod(candidates-position+track.length/2, ...
                track.length)-track.length/2);
            allowed = allowed & separation>=1.5*p.detour_length;
        end
        index = find(allowed,1);
        if isempty(index)
            obstacles.spawn_time(j) = time+p.T;
            obstacles.spawn_step(j) = k+1;
            if ~deferred(j)
                events(end+1) = struct('time',time,'type','deferred_spawn', ...
                    'detail',sprintf('Obstacle %d waits for a separated exposure ahead',j)); %#ok<AGROW>
                deferred(j) = true;
            end
            continue
        end
        query = candidates(index);
        obstacles.X(j) = interp1([track.s;track.length],[track.X(:);track.X(1)],query);
        obstacles.Y(j) = interp1([track.s;track.length],[track.Y(:);track.Y(1)],query);
        obstacles.spawn_time(j) = time;
        obstacles.spawn_step(j) = k;
        [~,obstacles.idx(j)] = min(abs(track.s-query));
        events(end+1) = struct('time',time,'type','spawn', ...
            'detail',sprintf('Obstacle %d exposed ahead',j)); %#ok<AGROW>
    end
    visible = obstacles(obstacles.spawn_time<=time,:);
    end
    stop = false;
    if independent_world
        [~,~,contact] = vehicle_clearance(x,physical_obstacles,track,p);
        if contact<=0
            termination = 'collision_on_spawn';
            stop = true;
            warning('mpc:CollisionOnSpawn','An independent world obstacle overlaps the vehicle at %.2f s.',time);
        end
    end
    map_changed = height(visible) ~= old_visible_count;
    if independent_world, map_changed = ~isequal(visible.id,old_visible_ids); end
    if moving_world
        geometry = visible{:,{'id','X','Y'}};
        map_changed = ~isequal(geometry,old_visible_geometry);
    end
    if ~stop && map_changed
        [gap,edge] = vehicle_clearance(x,visible,track,p);
        if gap < p.clearance || edge < p.clearance
            termination = 'unsafe_spawn';
            if moving_world, termination = 'unsafe_observation'; end
            stop = true;
            warning('mpc:UnsafeSpawn','Obstacle exposure violates current footprint clearance at %.2f s.',time);
        else
            try
                path = plan_reference_path(track,visible,p);
                plans(end+1) = struct('time',time,'path',path); %#ok<AGROW>
                events(end+1) = struct('time',time,'type','replan', ...
                    'detail',sprintf('%d visible obstacles',height(visible))); %#ok<AGROW>
            catch exception
                if ~strcmp(exception.identifier,'mpc:InfeasiblePath')
                    rethrow(exception);
                end
                braking = true;
                events(end+1) = struct('time',time,'type','infeasible_replan', ...
                    'detail',exception.message); %#ok<AGROW>
                warning('mpc:InfeasibleReplan','%s at %.2f s.',exception.message,time);
            end
        end
        old_visible_count = height(visible);
        if independent_world, old_visible_ids = visible.id; end
        if moving_world, old_visible_geometry = geometry; end
    end
    [reference,current] = make_reference(path,x,p);
    planning_time = toc(step_start);
    start = tic;
    violation = 0;
    if stop
        u = previous;
        flag = -99; iterations = 0; variables = 0;
    elseif braking
        [sequence,valid] = braking_sequence(x,previous,visible,track,p);
        u = sequence(:,1);
        flag = -10; iterations = 0; variables = p.Np;
        stop = ~valid;
    elseif strcmp(controller,'nonlinear')
        solver_called(k) = true;
        holder.ref = reference;
        holder.previous = previous;
        holder.track = track;
        holder.obstacles = visible;
        [input,nloptions,info] = nlmpcmove(model,x',previous',[],[],nloptions);
        u = input(:);
        flag = info.ExitFlag;
        iterations = info.Iterations;
        variables = NaN;
        sequence = info.MVopt(1:p.Np,:)';
    else
        solver_called(k) = true;
        [sequence,diagnostic] = mpc_solve(x,previous,reference,track,visible,p,controller,warm);
        u = sequence(:,1);
        flag = diagnostic.exitflag;
        iterations = diagnostic.iterations;
        variables = diagnostic.variables;
        violation = diagnostic.constraint_violation;
        if strcmp(controller,'laguerre') && flag>0
            coefficients(k,:) = diagnostic.eta';
        end
        warm = [sequence(:,2:end),sequence(:,end)];
    end
    elapsed = toc(start);
    validation_start = tic;
    if ~stop && ~braking
        [valid,~] = check_control_sequence(x,previous,sequence,visible,track,p);
        prediction_valid(k) = valid;
        if flag <= 0 || ~valid
            events(end+1) = struct('time',time,'type','controller_failure', ...
                'detail',sprintf('exitflag=%d, nonlinear_validation=%d',flag,valid)); %#ok<AGROW>
            warning('mpc:Fallback','%s requires a checked braking fallback at %.2f s (exitflag %d).', ...
                controller,time,flag);
            braking = true;
            [sequence,valid] = braking_sequence(x,previous,visible,track,p);
            u = sequence(:,1);
            stop = ~valid;
        end
    end
    [~,gap,road,rectangle,contact] = check_motion(x,u,visible,track,p);
    observed_clearance(k) = gap;
    if moving_world
        [gap,road,rectangle,contact] = check_moving_motion(x,u,world,time,track,p);
        world_counts(k) = height(obstacles);
    elseif independent_world
        % World truth monitors physics only; it is never used to choose or veto u.
        [~,gap,road,rectangle,contact] = check_motion(x,u,physical_obstacles,track,p);
        world_counts(k) = height(physical_obstacles);
    else
        world_counts(k) = height(visible);
        total_spawned(k) = height(visible);
    end
    confirmed_contact(k) = contact;
    if stop
        if ~independent_world && ~moving_world, physical_obstacles = visible; end
        [~,~,current_contact] = vehicle_clearance(x,physical_obstacles,track,p);
        confirmed_contact(k) = current_contact<=0;
    end
    if stop && strcmp(termination,'completed')
        termination = 'no_safe_control';
        warning('mpc:NoSafeControl','No verified collision-free braking trajectory at %.2f s.',time);
    end
    validation_time = toc(validation_start);
    [~,track_error] = project_path(track,x(1:2));
    [~,path_error] = project_path(path,x(1:2));
    if isempty(visible)
        active = 0; distance = NaN;
    else
        [distance,active] = min(hypot(visible.X-x(1),visible.Y-x(2)));
    end
    status(k) = "tracking";
    if height(visible)>0, status(k) = "avoiding"; end
    if braking, status(k) = "braking"; end
    if stop, status(k) = string(termination); end
    ids = (1:height(visible))';
    if ismember('id',visible.Properties.VariableNames), ids = visible.id; end
    known_ids(k) = string(jsonencode(ids'));
    values(k,:) = [(k-1)*p.T,x',u',current,reference(1,1:2), ...
        track_error,path_error,elapsed,flag,iterations,variables,braking,gap, ...
        road,rectangle,height(visible),active,distance,violation,validation_time, ...
        planning_time];
    if stop
        step_times(k) = toc(step_start);
        break
    end
    x = bicycle_step(x,u,p);
    previous = u;
    step_times(k) = toc(step_start);
    if (independent_world || moving_world) && contact
        termination = 'collision';
        status(k) = "collision";
        events(end+1) = struct('time',time+p.T,'type','collision', ...
            'detail','Physical monitor confirmed rectangle/obstacle contact after applying the command'); %#ok<AGROW>
        warning('mpc:Collision','Physical contact occurred during the step ending at %.2f s.',time+p.T);
        break
    end
    if braking && x(4)<1e-5
        termination = 'safely_stopped';
        break
    end
end
if ~strcmp(termination,'completed'), status(k) = string(termination); end
names = {'t','X','Y','psi','v','delta','a','X_ref','Y_ref','psi_ref','v_ref', ...
    'X_target','Y_target','track_cte','path_error','solve_time','exitflag', ...
    'iterations','variables','fallback','min_clearance','road_clearance', ...
    'rectangle_clearance','visible_obstacles','active_obstacle','min_obs_dist', ...
    'constraint_violation','validation_time','planning_time'};
logs = array2table(values(1:k,:),'VariableNames',names);
logs.status = status(1:k);
logs.applied = true(k,1);
if stop, logs.applied(end) = false; end
logs.step_time = step_times(1:k);
logs.solver_called = solver_called(1:k);
logs.prediction_valid = prediction_valid(1:k);
logs.world_obstacles = world_counts(1:k);
logs.new_detections = new_detections(1:k);
logs.new_removals = new_removals(1:k);
logs.removal_detections = removal_detections(1:k);
logs.total_spawned = total_spawned(1:k);
logs.total_removed = total_removed(1:k);
logs.observed_clearance = observed_clearance(1:k);
logs.known_obstacle_ids = known_ids(1:k);
logs.confirmed_contact = confirmed_contact(1:k);
if strcmp(controller,'laguerre')
    for mode = 1:p.laguerre_order
        logs.(sprintf('eta_delta_%d',mode)) = coefficients(1:k,2*mode-1);
        logs.(sprintf('eta_accel_%d',mode)) = coefficients(1:k,2*mode);
    end
end
metrics = evaluate_run(logs,x,track,p);
metrics.success = metrics.success && strcmp(termination,'completed') && ~braking;
metrics.termination = termination;
metrics.pending_obstacles = sum(~isfinite(obstacles.X));
if moving_world
    world_metadata = world.metadata();
    metrics.spawned_obstacles = height(obstacles);
    metrics.detected_obstacles = height(visible);
elseif independent_world
    world_metadata = world.metadata();
    metrics.spawned_obstacles = height(obstacles);
    metrics.detected_obstacles = sum(isfinite(obstacles.detection_time));
    metrics.undetected_obstacles = sum(isnan(obstacles.detection_time));
    metrics.removed_obstacles = sum(isfinite(obstacles.removal_time));
    metrics.confirmed_removals = sum(isfinite(obstacles.removal_detection_time));
    metrics.active_obstacles = sum(isnan(obstacles.removal_time));
else
    world_metadata = struct('mode','controlled', ...
        'spawn_policy','car_relative_placement_with_explicit_deferral_for_appearing');
end
if metrics.pending_obstacles>0
    metrics.success = false;
    if strcmp(metrics.termination,'completed')
        metrics.termination = 'pending_exposures';
    end
    warning('mpc:PendingObstacles','Run ended with %d requested exposures still pending.', ...
        metrics.pending_obstacles);
end
recorded_seed = options.Seed;
if independent_world, recorded_seed = world_metadata.seed; end
result = struct('controller',controller,'scenario',scenario,'parameters',p, ...
    'seed',recorded_seed,'track',track,'path',path,'obstacles',obstacles, ...
    'logs',logs,'initial_state',initial,'final_state',x,'metrics',metrics, ...
    'events',events,'plans',plans,'world',world_metadata, ...
    'requested_duration',N*p.T,'run_dir','');
if moving_world
    result.obstacle_history = vertcat(obstacle_history{1:k});
end
if options.Save
    result.run_dir = save_mpc_run(result,options.OutputRoot);
end
if options.Plot
    figure_handle = plot_results(result);
    if options.Save
        exportgraphics(figure_handle,fullfile(result.run_dir,'summary.png'));
    end
end
fprintf('%s/%s: RMS track CTE %.2f cm, speed RMSE %.3f m/s, saved: %s\n', ...
    controller,scenario,100*metrics.track_cte_rms,metrics.speed_rmse,result.run_dir);
end
