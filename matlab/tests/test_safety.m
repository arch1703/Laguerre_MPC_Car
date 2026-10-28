function tests = test_safety
tests = functiontests(localfunctions);
end

function testSharedGeometry(testCase)
p = parameters();
track = generate_track(p);
x = [2;0;pi/2;1.2];
center = x(1:2)+p.body_offset*[cos(x(3));sin(x(3))];
obstacles = table(center(1)+0.4,center(2),0.12,'VariableNames',{'X','Y','R'});
[gap,~,rectangle] = vehicle_clearance(x,obstacles,track,p);
verifyEqual(testCase,gap,0.4-p.body_radius-0.12,'AbsTol',1e-12);
verifyGreaterThan(testCase,rectangle,gap);
end

function testInterSampleCollision(testCase)
p = parameters(); p.T = 0.5; p.road_half_width = 3;
track = generate_track(p);
obstacles = table(2,0.75,0.08,'VariableNames',{'X','Y','R'});
[safe,gap] = check_motion([2;0;pi/2;3],[0;0],obstacles,track,p);
verifyFalse(testCase,safe);
verifyLessThan(testCase,gap,0);
end

function testCheckedBraking(testCase)
p = parameters(); track = generate_track(p);
empty = make_demo_obstacles(track,p,'tracking',15,21);
x = [2;0;pi/2;1.2];
[sequence,safe] = braking_sequence(x,[0;0],empty,track,p);
verifyTrue(testCase,safe);
for i = 1:p.Np
    x = bicycle_step(x,sequence(:,i),p);
end
verifyEqual(testCase,x(4),0,'AbsTol',1e-6);
blocked = table(2,0.12,0.12,'VariableNames',{'X','Y','R'});
[~,safe] = braking_sequence([2;0;pi/2;1.2],[0;0],blocked,track,p);
verifyFalse(testCase,safe);
end

function testUnavoidableExposure(testCase)
obstacles = table(2,0.25,0.12,0.1,'VariableNames',{'X','Y','R','spawn_time'});
result = run_mpc_demo('laguerre','appearing',Duration=1,Save=false,Obstacles=obstacles);
verifyFalse(testCase,result.metrics.success);
verifyEqual(testCase,result.metrics.termination,'unsafe_spawn');
verifyFalse(testCase,result.logs.applied(end));
verifyLessThan(testCase,result.metrics.duration,1);
end

function testStaticAvoidance(testCase)
for controller = {'laguerre','ltv'}
    result = run_mpc_demo(controller{1},'static',Duration=15,Save=false);
    verifyTrue(testCase,result.metrics.success);
    verifyGreaterThanOrEqual(testCase,result.metrics.min_clearance,0.05);
    verifyGreaterThanOrEqual(testCase,result.metrics.min_road_clearance,0.05);
    verifyEqual(testCase,result.metrics.collision_samples,0);
    verifyEqual(testCase,result.metrics.fallback_steps,0);
    verifyGreaterThan(testCase,result.metrics.laps,1);
    verifyLessThan(testCase,result.metrics.path_error_rms,0.02);
end
end

function testAppearingAvoidance(testCase)
result = run_mpc_demo('laguerre','appearing',Duration=15,Save=false);
verifyTrue(testCase,result.metrics.success);
verifyEqual(testCase,max(result.logs.visible_obstacles),3);
verifyEqual(testCase,result.metrics.collision_samples,0);
verifyGreaterThanOrEqual(testCase,result.metrics.min_clearance,0.05);
verifyLessThan(testCase,result.metrics.path_error_rms,0.02);
end

function testInvalidInputs(testCase)
verifyError(testCase,@() run_mpc_demo('laguerre','tracking', ...
    Parameters=struct('not_a_parameter',1),Save=false),'mpc:UnknownParameter');
verifyError(testCase,@() run_mpc_demo('laguerre','tracking', ...
    Parameters=struct('road_half_width',0.1),Save=false),'mpc:InvalidRoad');
verifyError(testCase,@() run_mpc_demo('laguerre','appearing', ...
    Obstacles=table(0,'VariableNames',{'X'}),Save=false),'mpc:InvalidObstacles');
end

function testUnfulfilledExposuresAreNotSuccess(testCase)
result = run_mpc_demo('laguerre','appearing',Duration=0.2,Save=false, ...
    Parameters=struct('detour_length',10));
verifyFalse(testCase,result.metrics.success);
verifyEqual(testCase,result.metrics.termination,'pending_exposures');
verifyEqual(testCase,result.metrics.pending_obstacles,3);
verifyTrue(testCase,all(result.logs.applied));
end

function testUnreachableCurvatureIsRejected(testCase)
p = parameters(); p.track_b = 0.6;
track = generate_track(p);
obstacles = make_demo_obstacles(track,p,'tracking',15,21);
verifyError(testCase,@() plan_reference_path(track,obstacles,p),'mpc:InfeasiblePath');
end

function testNoSafeControlDoesNotAdvancePlant(testCase)
initial = [5;0;pi/2;1.2];
result = run_mpc_demo('laguerre','tracking',Duration=1,Save=false,InitialState=initial);
verifyFalse(testCase,result.metrics.success);
verifyEqual(testCase,result.metrics.termination,'no_safe_control');
verifyEqual(testCase,result.final_state,initial);
verifyFalse(testCase,result.logs.applied(end));
end
