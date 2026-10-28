function tests = test_boundary_uncertainty
tests = functiontests(localfunctions);
end

function testBoundedMeasurementsAndCorridorShrinkage(testCase)
p = parameters(); track = make_boundary_track('chicane',p);
world = CrossingObstacleWorld(track); truth = world.snapshot();
pose = [truth.X(1);truth.Y(1);0;0];
c = boundary_experiment_parameters(struct('line_noise',0.02,'obstacle_noise',0.03));
sensor = BoundarySensor(c);
[corridor,measured,lines] = sensor.observe(track,world,pose,0);
[~,ideal] = sense_track_lines(track,pose,3.5);
columns = {'left_X','left_Y','right_X','right_Y'};
verifyLessThanOrEqual(testCase,max(abs(lines{:,columns}-ideal{:,columns}),[],'all'),0.02);
[~,indices] = ismember(measured.id,truth.id);
verifyLessThanOrEqual(testCase,max(abs(measured{:,{'X','Y'}}-truth{indices,{'X','Y'}}),[],'all'),0.03);
verifyEqual(testCase,measured.position_bound,sqrt(2)*0.03*ones(height(measured),1));
verifyEqual(testCase,corridor.half_width, ...
    hypot(lines.left_X-lines.right_X,lines.left_Y-lines.right_Y)/2-sqrt(2)*0.02, ...
    'AbsTol',1e-12);
end

function testDrawsAreIndexedIndependentlyOfPastPerception(testCase)
p = parameters(); track = make_boundary_track('kidney',p);
c = struct('line_noise',0.01,'obstacle_noise',0.01);
a = BoundarySensor(c); b = BoundarySensor(c);
wa = CrossingObstacleWorld(track); wb = CrossingObstacleWorld(track);
pose = [track.X(1);track.Y(1);track.psi(1);0];
a.observe(track,wa,pose,0);
b.observe(track,wb,[track.X(201);track.Y(201);track.psi(201);0],0);
wa.advance(0.05); wb.advance(0.05);
[~,ma,la] = a.observe(track,wa,pose,0.05);
[~,mb,lb] = b.observe(track,wb,pose,0.05);
verifyEqual(testCase,la,lb);
ma = ma(ma.last_seen==0.05,1:7); mb = mb(mb.last_seen==0.05,1:7);
verifyEqual(testCase,ma,mb);
end

function testLatencyAndLostLinesPreserveAcquisitionTimes(testCase)
p = parameters(); track = make_boundary_track('oval',p);
world = CrossingObstacleWorld(track);
sensor = BoundarySensor(struct('latency_steps',2,'line_loss_start',0.05,'line_loss_end',20));
pose = [track.X(1);track.Y(1);track.psi(1);0];
[~,~,initial] = sensor.observe(track,world,pose,0);
for time = 0.05:0.05:0.6
    world.advance(time);
    [~,measured,lines,state_history,state] = sensor.observe(track,world,pose,time); %#ok<ASGLU>
    verifyLessThanOrEqual(testCase,state.delivery_time,time);
    verifyTrue(testCase,all(measured.last_seen<=state.delivery_time));
end
verifyEqual(testCase,lines.left_X,initial.left_X);
verifyEqual(testCase,lines.capture_time,zeros(height(lines),1));
verifyEqual(testCase,state.line_age,0.6,'AbsTol',1e-12);
verifyEqual(testCase,state.delivery_time,0.5,'AbsTol',1e-12);
verifyTrue(testCase,state.line_stale);
end

function testUnknownClassesAndOutOfRangeMemory(testCase)
p = parameters(); track = make_boundary_track('oval',p);
world = CrossingObstacleWorld(track);
truth = world.snapshot(); pose = [truth.X(2);truth.Y(2);0;0];
sensor = BoundarySensor(struct('uncertain_classification',true));
[~,measured] = sensor.observe(track,world,pose,0);
old_ids = measured.id;
verifyTrue(testCase,any(measured.id==2));
verifyEqual(testCase,measured.speed_bound,world.speed_bound()*ones(height(measured),1));
world.advance(0.05);
[~,remembered,~,~,state] = sensor.observe(track,world, ...
    [track.X(1);track.Y(1);track.psi(1);0],0.05);
remembered = remembered(ismember(remembered.id,old_ids),:);
verifyEqual(testCase,remembered.last_seen,zeros(height(remembered),1));
verifyTrue(testCase,all(remembered.outside_visible));
verifyFalse(testCase,state.obstacle_stale);
lossy = BoundarySensor(struct('miss_probability',1));
[~,none,~,~,state] = lossy.observe(track,world,pose,0.05);
verifyEmpty(testCase,none);
verifyFalse(testCase,state.complete_detection);
end

function testVisibilityRequiresWholeStoppingTube(testCase)
p = parameters(); sensing = struct('complete_detection',true,'range_center',[0;0], ...
    'range',3.5,'range_age',0,'unseen_radius_bound',0.1,'unseen_speed_bound',0.3);
verifyTrue(testCase,check_sensor_visibility([0;0;0;0.8],zeros(2,12),p,sensing));
verifyFalse(testCase,check_sensor_visibility([3;0;0;0.8],zeros(2,24),p,sensing));
sensing.range_age = 20;
verifyFalse(testCase,check_sensor_visibility(zeros(4,1),zeros(2,24),p,sensing));
end

function testLossCannotPassByWaitingIndefinitely(testCase)
result = run_boundary_demo('chicane',Duration=15,Save=false, ...
    Experiment=struct('line_loss_start',0.05,'line_loss_end',20));
verifyEqual(testCase,result.metrics.termination,'stalled');
verifyFalse(testCase,result.metrics.success);
verifyFalse(testCase,result.metrics.progress_sufficient);
verifyTrue(testCase,result.metrics.safety_pass);
verifyGreaterThanOrEqual(testCase,result.metrics.max_stopped_duration,12);
verifyEqual(testCase,result.metrics.required_laps,0.35*0.8*15/result.track.length,'AbsTol',1e-12);
verifyTrue(testCase,all(result.logs.stopping_reserve(result.logs.applied)));
verifyTrue(testCase,all(result.logs.step_time>=result.logs.online_time));
end

function testLtvReplayPreservesPerturbedExperiment(testCase)
directory = tempname; mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
original = run_boundary_demo('oval',Duration=0.5,Controller='ltv',OutputRoot=directory, ...
    Experiment=struct('line_noise',0.01,'obstacle_noise',0.01,'latency_steps',2, ...
    'plant_wheelbase_scale',1.1,'plant_acceleration_scale',0.9));
replay = replay_mpc_run(original.run_dir,'ltv',Save=false);
verifyEqual(testCase,replay.experiment,original.experiment);
verifyEqual(testCase,replay.parameters.L,0.25);
verifyEqual(testCase,replay.logs{:,{'X','Y','delta','a'}}, ...
    original.logs{:,{'X','Y','delta','a'}},'AbsTol',1e-9);
verifyEqual(testCase,replay.measurement_history,original.measurement_history);
verifyTrue(testCase,isfile(fullfile(original.run_dir,'measurement_history.csv')));
end

function testInvalidConfigurationIsExplicit(testCase)
verifyError(testCase,@() boundary_experiment_parameters(struct('hidden_future',1)), ...
    'mpc:UnknownExperimentParameter');
verifyError(testCase,@() boundary_experiment_parameters(struct('line_loss_start',2,'line_loss_end',1)), ...
    'mpc:InvalidExperiment');
end
