function tests = test_moving_world
tests = functiontests(localfunctions);
end

function testMotionIndependentOfObserver(testCase)
track = generate_track(parameters());
first = MovingObstacleWorld(track);
second = MovingObstacleWorld(track);
initial = first.snapshot();
for time = 0:0.1:2
    first.advance(time); second.advance(time);
    first.observe([0;0;0;0],time);
    second.observe([100;100;0;0],time);
end
verifyEqual(testCase,first.snapshot(),second.snapshot());
final = first.snapshot();
verifyGreaterThan(testCase,min(hypot(final.X-initial.X,final.Y-initial.Y)),0.3);
end

function testMeasurementsFreezeOutsideRangeAndDoNotLeakMotion(testCase)
world = MovingObstacleWorld(generate_track(parameters()));
truth = world.snapshot();
[known,~] = world.observe([truth.X(1);truth.Y(1);0;0],0);
verifyFalse(testCase,any(ismember({'speed','velocity','origins','future'}, ...
    known.Properties.VariableNames)));
first = known(known.id==1,:);
world.advance(1);
[known,history] = world.observe([100;100;0;0],1);
verifyEqual(testCase,known(known.id==1,:),first);
verifyFalse(testCase,any(history.sensed));
current = world.snapshot();
verifyGreaterThan(testCase,hypot(current.X(1)-first.X,current.Y(1)-first.Y),0.15);
[known,history] = world.observe([current.X(1);current.Y(1);0;0],1);
verifyEqual(testCase,known.X(known.id==1),current.X(1));
verifyEqual(testCase,known.last_seen(known.id==1),1);
verifyTrue(testCase,history.sensed(1));
end

function testTruthMonitorDetectsObstacleMotion(testCase)
p = parameters(); track = generate_track(p);
world = MovingObstacleWorld(track);
p.T = 3;
truth = world.snapshot(3);
[~,~,~,heading] = project_path(track,[truth.X(1),truth.Y(1)]);
x = [truth.X(1);truth.Y(1);heading;0];
[~,~,initial_box] = vehicle_clearance(x,world.snapshot(0),track,p);
verifyGreaterThan(testCase,initial_box,0);
[~,~,~,contact] = check_moving_motion(x,[0;0],world,0,track,p);
verifyTrue(testCase,contact);
end

function testMovingRunAndReplay(testCase)
directory = tempname;
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
result = run_mpc_demo('laguerre','moving',Duration=1, ...
    Parameters=struct('Np',10,'Nc',5),OutputRoot=directory);
verifyTrue(testCase,result.metrics.success);
verifyEqual(testCase,height(result.obstacle_history),3*height(result.logs));
verifyTrue(testCase,isfile(fullfile(result.run_dir,'obstacle_history.csv')));
verifyGreaterThan(testCase,numel(result.plans),2);
replay = replay_mpc_run(result.run_dir,'laguerre',Save=false);
verifyEqual(testCase,replay.obstacle_history,result.obstacle_history);
verifyEqual(testCase,replay.logs{:,{'X','Y','delta','a'}}, ...
    result.logs{:,{'X','Y','delta','a'}},'AbsTol',1e-10);
end
