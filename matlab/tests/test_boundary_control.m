function tests = test_boundary_control
tests = functiontests(localfunctions);
end

function testOpenProjectionDoesNotCreateClosingEdge(testCase)
path = path_geometry([0,0;2,0;2,2],false);
[~,distance] = project_path(path,[1,1]);
verifyEqual(testCase,distance,1,'AbsTol',1e-12);
p = parameters(); path.v = ones(3,1);
[reference,~] = make_reference(path,[2;1.99;pi/2;1],p);
verifyEqual(testCase,reference(:,1:2),repmat([2,2],p.Np,1),'AbsTol',1e-12);
end

function testBoundarySensorIsLocalAndLineDerived(testCase)
p = parameters(); track = make_boundary_track('chicane',p);
pose = [track.X(1);track.Y(1);track.psi(1);0.8];
[local,measurements] = sense_track_lines(track,pose,3.5);
verifyFalse(testCase,local.closed);
verifyLessThan(testCase,local.N,track.N/2);
verifyLessThanOrEqual(testCase,max(vecnorm(local.left-pose(1:2)',2,2)),3.5);
verifyLessThanOrEqual(testCase,max(vecnorm(local.right-pose(1:2)',2,2)),3.5);
verifyEqual(testCase,[local.X,local.Y],(local.left+local.right)/2,'AbsTol',1e-12);
verifyEqual(testCase,height(measurements),local.N);
perturbed = track;
perturbed.left(150:200,:) = perturbed.left(150:200,:)+100;
perturbed.right(150:200,:) = perturbed.right(150:200,:)+100;
[other,~] = sense_track_lines(perturbed,pose,3.5);
verifyEqual(testCase,other,local);
end

function testBoundaryClearanceAndDifferentShapes(testCase)
p = parameters();
for kind = {'oval','kidney','chicane'}
    track = make_boundary_track(kind{1},p);
    pose = [track.X(1);track.Y(1);track.psi(1);0];
    [~,gap] = vehicle_clearance(pose,table(),track,p);
    verifyGreaterThan(testCase,gap,0.5);
    outside = pose;
    outside(1:2) = pose(1:2)+2*[-sin(pose(3));cos(pose(3))];
    [~,gap] = vehicle_clearance(outside,table(),track,p);
    verifyLessThan(testCase,gap,0);
end
end

function testCrossingMotionAndObservationBoundary(testCase)
p = parameters(); track = make_boundary_track('chicane',p);
first = CrossingObstacleWorld(track); second = CrossingObstacleWorld(track);
initial = first.snapshot(0); later = first.snapshot(7);
delta = [later.X(1)-initial.X(1),later.Y(1)-initial.Y(1)];
[~,~,~,heading] = project_path(track,[later.X(1),later.Y(1)]);
verifyLessThan(testCase,abs(delta*[cos(heading);sin(heading)]),0.02);
verifyGreaterThan(testCase,norm(delta),1.2);
for time = 0:0.1:1
    first.advance(time); second.advance(time);
    first.observe([0;0;0;0],time,3.5);
    second.observe([100;100;0;0],time,3.5);
end
verifyEqual(testCase,first.snapshot(),second.snapshot());
truth = first.snapshot();
[observed,~] = first.observe([truth.X(1);truth.Y(1);0;0],1,3.5);
verifyEqual(testCase,observed.Properties.VariableNames, ...
    {'id','X','Y','R','last_seen','speed_bound'});
verifyTrue(testCase,all(observed.last_seen==1));
verifyLessThan(testCase,height(first.snapshot(27)),height(first.snapshot(15)));
end

function testInaccurateLinearizationIsExplicitlyRejected(testCase)
p = parameters(); p.Np = 12; p.Nc = 6;
p.linearization_guard = true; p.linearization_iterations = 1;
p.linearization_position_tolerance = 1e-12;
p.linearization_heading_tolerance = 1e-12;
track = generate_track(p); track.v = p.v_tgt*ones(track.N,1);
x = [track.X(1);track.Y(1);track.psi(1);p.v_tgt];
reference = make_reference(track,x,p);
[~,diagnostic] = mpc_solve(x,zeros(2,1),reference,track,table(),p, ...
    'laguerre',zeros(2,p.Nc));
verifyEqual(testCase,diagnostic.exitflag,-20);
verifyGreaterThan(testCase,diagnostic.model_position_error,1e-12);
end

function testBrakeEndsWithZeroAcceleration(testCase)
p = parameters(); p.Np = 24; p.brake_terminal_acceleration = true;
track = make_boundary_track('oval',p);
x = [track.X(1);track.Y(1);track.psi(1);0.8];
[corridor,~] = sense_track_lines(track,x,3.5);
[sequence,safe] = braking_sequence(x,zeros(2,1),table(),corridor,p);
verifyTrue(testCase,safe);
for i = 1:size(sequence,2), x = bicycle_step(x,sequence(:,i),p); end
verifyEqual(testCase,x(4),0,'AbsTol',1e-6);
verifyEqual(testCase,sequence(2,end),0,'AbsTol',1e-6);
end

function testCrossingYieldAndResume(testCase)
result = run_boundary_demo('chicane',Duration=12,Save=false);
verifyTrue(testCase,result.metrics.success);
verifyGreaterThan(testCase,result.metrics.yielding_steps,10);
verifyGreaterThan(testCase,result.metrics.resume_events,0);
verifyLessThan(testCase,min(result.logs.v),1e-4);
verifyGreaterThan(testCase,result.logs.v(end),0.5);
accepted = result.logs.solver_called & result.logs.prediction_valid;
verifyTrue(testCase,all(result.logs.model_position_error(accepted)<= ...
    result.parameters.linearization_position_tolerance));
verifyTrue(testCase,all(result.logs.model_heading_error(accepted)<= ...
    result.parameters.linearization_heading_tolerance));
verifyEqual(testCase,result.metrics.confirmed_contact_steps,0);
end

function testBoundaryPersistenceAndReplay(testCase)
directory = tempname; mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
original = run_boundary_demo('kidney',Duration=0.5,OutputRoot=directory);
verifyTrue(testCase,isfile(fullfile(original.run_dir,'line_history.csv')));
replay = replay_mpc_run(original.run_dir,'laguerre',Save=false);
verifyEqual(testCase,replay.logs{:,{'X','Y','delta','a'}}, ...
    original.logs{:,{'X','Y','delta','a'}},'AbsTol',1e-9);
verifyError(testCase,@() replay_mpc_run(original.run_dir,'nonlinear',Save=false), ...
    'mpc:InvalidController');
end

function testKidneyStoppingReserveAndStableResumption(testCase)
result = run_boundary_demo('kidney',Duration=40,Save=false);
verifyTrue(testCase,result.metrics.success);
verifyGreaterThan(testCase,result.metrics.laps,1);
verifyEqual(testCase,result.metrics.fallback_steps,0);
verifyGreaterThanOrEqual(testCase,result.metrics.resume_events,2);
times = [result.events(strcmp({result.events.type},'resume')).time];
for time = times
    index = find(abs(result.logs.t-time)<1e-8);
    verifyLessThan(testCase,result.logs.v(index),1e-4);
    verifyLessThan(testCase,abs(result.logs.a(index-1)),1e-3);
end
end
