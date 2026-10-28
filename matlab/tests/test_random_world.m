function tests = test_random_world
tests = functiontests(localfunctions);
end

function testWorldIsIndependentOfObserver(testCase)
track = generate_track(parameters());
config = struct('mean_interval',0.4,'max_obstacles',4,'sensor_range',10);
first = RandomObstacleWorld(track,config,17);
second = RandomObstacleWorld(track,config,17);
verifyEqual(testCase,height(first.snapshot()),0);
for time = 0:0.1:5
    first.advance(time);
    second.advance(time);
    first.observe([0;0;0;0],time);
    second.observe([1000;1000;0;0],time);
end
a = first.snapshot(); b = second.snapshot();
physical = {'id','X','Y','R','spawn_time','requested_spawn_time'};
verifyGreaterThan(testCase,height(a),0);
verifyEqual(testCase,a(:,physical),b(:,physical));
verifyTrue(testCase,all(isfinite(a.detection_time)));
verifyTrue(testCase,all(isnan(b.detection_time)));
verifyTrue(testCase,all(a.detection_time>=a.spawn_time));
end

function testSensorAndRememberedMap(testCase)
track = generate_track(parameters());
world = RandomObstacleWorld(track,struct('max_obstacles',1,'sensor_range',0.1),29);
world.advance(100);
truth = world.snapshot();
[known,new] = world.observe([1000;1000;0;0],100);
verifyEqual(testCase,height(known),0);
verifyEqual(testCase,height(new),0);
pose = [truth.X(1);truth.Y(1);0;0];
[known,new] = world.observe(pose,100);
verifyEqual(testCase,known.id,truth.id);
verifyEqual(testCase,height(new),1);
[known,new] = world.observe([1000;1000;0;0],100);
verifyEqual(testCase,height(known),1);
verifyEqual(testCase,height(new),0);
verifyEqual(testCase,known.detection_time,100);
end

function testHiddenTruthDoesNotChangeControls(testCase)
p = parameters(); track = generate_track(p);
config = struct('mean_interval',0.01,'max_obstacles',1, ...
    'sensor_range',0.01,'lateral_spread',0);
probe = RandomObstacleWorld(track,config,53);
probe.advance(1);
truth = probe.snapshot();
[~,index] = min((track.X(:)+truth.X(1)).^2+(track.Y(:)+truth.Y(1)).^2);
initial = [track.X(index);track.Y(index);track.psi(index);p.v_tgt];
overrides = struct('Np',10,'Nc',5);
with_hidden = run_mpc_demo('laguerre','random',Duration=0.5,Save=false, ...
    Parameters=overrides,WorldSeed=53,WorldParameters=config,InitialState=initial);
config.max_obstacles = 0;
empty = run_mpc_demo('laguerre','random',Duration=0.5,Save=false, ...
    Parameters=overrides,WorldSeed=53,WorldParameters=config,InitialState=initial);
verifyEqual(testCase,max(with_hidden.logs.world_obstacles),1);
verifyEqual(testCase,max(with_hidden.logs.visible_obstacles),0);
verifyEqual(testCase,with_hidden.logs{:,{'X','Y','psi','v','delta','a'}}, ...
    empty.logs{:,{'X','Y','psi','v','delta','a'}},'AbsTol',1e-12);
verifyEqual(testCase,numel(with_hidden.plans),1);
end

function testFixedSeedDiagnosticsAndReplay(testCase)
directory = tempname;
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
config = struct('mean_interval',1,'max_obstacles',2);
result = run_mpc_demo('laguerre','random',Duration=6,Parameters=struct('Np',10,'Nc',5), ...
    WorldSeed=17,WorldParameters=config,OutputRoot=directory);
verifyGreaterThan(testCase,height(result.obstacles),0);
verifyEqual(testCase,result.world.seed,17);
verifyEqual(testCase,result.seed,17);
verifyTrue(testCase,all(result.obstacles.detection_time>=result.obstacles.spawn_time | ...
    isnan(result.obstacles.detection_time)));
replay = replay_mpc_run(result.run_dir,'laguerre',Save=false);
verifyEqual(testCase,replay.obstacles,result.obstacles);
verifyEqual(testCase,replay.logs{:,{'X','Y','delta','a','visible_obstacles'}}, ...
    result.logs{:,{'X','Y','delta','a','visible_obstacles'}},'AbsTol',1e-10);
verifyFalse(testCase,any(strcmp({result.events.type},'deferred_spawn')));
end

function testIndependentSpawnCollisionIsNotMoved(testCase)
track = generate_track(parameters());
config = struct('mean_interval',0.001,'max_obstacles',1);
probe = RandomObstacleWorld(track,config,13);
probe.advance(1);
truth = probe.snapshot();
[~,~,~,heading] = project_path(track,[truth.X,truth.Y]);
initial = [truth.X;truth.Y;heading;1.2];
result = run_mpc_demo('laguerre','random',Duration=0.1,Save=false, ...
    WorldSeed=13,WorldParameters=config,InitialState=initial);
verifyEqual(testCase,result.obstacles.X,truth.X,'AbsTol',1e-12);
verifyEqual(testCase,result.obstacles.Y,truth.Y,'AbsTol',1e-12);
verifyEqual(testCase,result.metrics.termination,'collision_on_spawn');
verifyFalse(testCase,result.metrics.success);
verifyEqual(testCase,result.metrics.confirmed_contact_steps,1);
verifyFalse(testCase,result.logs.applied(end));
end

function testNoDetectionNoReplan(testCase)
result = run_mpc_demo('laguerre','random',Duration=1,Save=false, ...
    WorldSeed=11,WorldParameters=struct('max_obstacles',0));
verifyTrue(testCase,result.metrics.success);
verifyEqual(testCase,numel(result.plans),1);
verifyEqual(testCase,sum(result.logs.new_detections),0);
verifyEqual(testCase,result.metrics.spawned_obstacles,0);
end

function testRejectScriptedInputInRandomMode(testCase)
scripted = table(2,0.2,0.12,1,'VariableNames',{'X','Y','R','spawn_time'});
verifyError(testCase,@() run_mpc_demo('laguerre','random',Obstacles=scripted, ...
    Save=false),'mpc:InvalidScenario');
verifyError(testCase,@() random_world_parameters(struct('radius_min',0.2,'radius_max',0.1)), ...
    'mpc:InvalidWorldParameters');
verifyError(testCase,@() random_world_parameters(struct('lifetime_min',1)), ...
    'mpc:InvalidWorldParameters');
verifyError(testCase,@() random_world_parameters(struct('lifetime_min',3,'lifetime_max',2)), ...
    'mpc:InvalidWorldParameters');
end

function testPrivateExpiryAndSensorConfirmedRemoval(testCase)
track = generate_track(parameters());
world = RandomObstacleWorld(track,struct('max_obstacles',1, ...
    'lifetime_min',1,'lifetime_max',1,'sensor_range',0.1),29);
world.advance(100);
truth = world.snapshot();
verifyTrue(testCase,isnan(truth.removal_time));
pose = [truth.X;truth.Y;0;0];
[known,new] = world.observe(pose,100);
verifyEqual(testCase,height(new),1);
verifyFalse(testCase,ismember('removal_time',known.Properties.VariableNames));
[~,removed] = world.advance(101);
verifyEqual(testCase,height(removed),1);
[known,new,cleared] = world.observe([1000;1000;0;0],101);
verifyEqual(testCase,height(known),1);
verifyEqual(testCase,height(new),0);
verifyEqual(testCase,height(cleared),0);
verifyTrue(testCase,isnan(known.removal_detection_time));
verifyFalse(testCase,ismember('removal_time',known.Properties.VariableNames));
world.advance(102);
[known,new,cleared] = world.observe(pose,102);
verifyEqual(testCase,height(known),0);
verifyEqual(testCase,height(new),0);
verifyEqual(testCase,height(cleared),1);
verifyEqual(testCase,cleared.removal_detection_time,102);
truth = world.snapshot();
verifyEqual(testCase,truth.removal_time,101);
verifyEqual(testCase,truth.removal_detection_time,102);
end

function testNeverDetectedExpiredObjectsStayUnknown(testCase)
track = generate_track(parameters());
world = RandomObstacleWorld(track,struct('max_obstacles',1, ...
    'lifetime_min',1,'lifetime_max',1),29);
world.advance(100);
truth = world.snapshot();
world.observe([1000;1000;0;0],100);
world.advance(101);
[known,new,cleared] = world.observe([truth.X;truth.Y;0;0],101);
verifyEqual(testCase,height(known),0);
verifyEqual(testCase,height(new),0);
verifyEqual(testCase,height(cleared),0);
end

function testTransientWorldIndependentOfObserver(testCase)
track = generate_track(parameters());
config = struct('mean_interval',0.4,'max_obstacles',12, ...
    'sensor_range',10,'lifetime_min',0.5,'lifetime_max',1.5);
first = RandomObstacleWorld(track,config,17);
second = RandomObstacleWorld(track,config,17);
for time = 0:0.1:8
    first.advance(time); second.advance(time);
    first.observe([0;0;0;0],time);
    second.observe([1000;1000;0;0],time);
end
a = first.snapshot(); b = second.snapshot();
physical = {'id','X','Y','R','spawn_time','requested_spawn_time','removal_time'};
verifyEqual(testCase,a(:,physical),b(:,physical));
verifyGreaterThan(testCase,sum(isfinite(a.removal_time)),4);
verifyTrue(testCase,all(a.removal_time-a.spawn_time>=config.lifetime_min));
verifyTrue(testCase,all(a.removal_time-a.spawn_time<config.lifetime_max+0.1+1e-12));
verifyTrue(testCase,all(isnan(b.detection_time)));
end

function testTransientReplayAndMapChanges(testCase)
directory = tempname;
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
config = struct('mean_interval',1,'max_obstacles',2, ...
    'lifetime_min',1.5,'lifetime_max',2,'sensor_range',10);
result = run_mpc_demo('laguerre','random',Duration=6,Parameters=struct('Np',10,'Nc',5), ...
    WorldSeed=17,WorldParameters=config,OutputRoot=directory);
verifyGreaterThan(testCase,result.metrics.removed_obstacles,0);
verifyGreaterThan(testCase,sum(result.logs.removal_detections),0);
verifyEqual(testCase,result.logs.world_obstacles, ...
    result.logs.total_spawned-result.logs.total_removed);
verifyEqual(testCase,numel(result.plans)-1, ...
    sum(result.logs.new_detections>0 | result.logs.removal_detections>0));
replay = replay_mpc_run(result.run_dir,'laguerre',Save=false);
verifyEqual(testCase,replay.obstacles,result.obstacles);
verifyEqual(testCase,replay.logs{:,{'X','Y','delta','a','world_obstacles','visible_obstacles'}}, ...
    result.logs{:,{'X','Y','delta','a','world_obstacles','visible_obstacles'}},'AbsTol',1e-10);
end

function testEqualCountReplacementReplans(testCase)
result = run_mpc_demo('laguerre','random',Duration=14.4,Save=false, ...
    Parameters=struct('Np',10,'Nc',5),WorldSeed=273210986, ...
    WorldParameters=struct('mean_interval',2,'max_obstacles',60, ...
    'lifetime_min',6,'lifetime_max',12));
index = find(abs(result.logs.t-14.3)<1e-8);
verifyEqual(testCase,numel(index),1);
verifyEqual(testCase,result.logs.visible_obstacles(index), ...
    result.logs.visible_obstacles(index-1));
verifyNotEqual(testCase,result.logs.known_obstacle_ids(index), ...
    result.logs.known_obstacle_ids(index-1));
verifyEqual(testCase,result.logs.new_detections(index),1);
verifyEqual(testCase,result.logs.removal_detections(index),1);
verifyTrue(testCase,any(abs([result.plans.time]-14.3)<1e-8));
verifyTrue(testCase,result.metrics.success);
end
