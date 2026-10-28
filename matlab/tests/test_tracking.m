function tests = test_tracking
tests = functiontests(localfunctions);
end

function testDiscreteNonlinearModel(testCase)
p = parameters();
[model,~,~] = create_nonlinear_mpc(p);
verifyFalse(testCase,model.Model.IsContinuousTime);
x = [2;0;pi/2;1.2];
u = [0.2;0.4];
verifyEqual(testCase,model.Model.StateFcn(x,u),bicycle_step(x,u,p),'AbsTol',1e-12);
end

function testArcLengthReference(testCase)
p = parameters();
path = generate_track(p);
path.v = p.v_tgt*ones(path.N,1);
for index = [1,100,200,399]
    x = [path.X(index);path.Y(index);path.psi(index);p.v_tgt];
    ref = make_reference(path,x,p);
    increments = vecnorm(diff([x(1:2)';ref(:,1:2)]),2,2);
    verifyEqual(testCase,increments,p.T*p.v_tgt*ones(p.Np,1),'AbsTol',2e-4);
    verifyLessThan(testCase,max(abs(diff([x(3);ref(:,3)]))),0.1);
end
end

function testPredictionMatchesRollout(testCase)
rng(7);
Np = 7; Nc = 3;
A = repmat(eye(4),1,1,Np)+0.01*randn(4,4,Np);
B = 0.1*randn(4,2,Np);
c = 0.01*randn(4,Np);
x = randn(4,1);
U = randn(2,Nc);
[Phi,Theta,C] = build_prediction_matrices(A,B,c,Np,Nc);
expected = zeros(4*Np,1);
state = x;
for i = 1:Np
    state = A(:,:,i)*state+B(:,:,i)*U(:,min(i,Nc))+c(:,i);
    expected((i-1)*4+(1:4)) = state;
end
verifyEqual(testCase,Phi*x+Theta*U(:)+C,expected,'AbsTol',1e-12);
end

function testLaguerreBasis(testCase)
basis = make_laguerre_basis(300,4,0.7);
verifyEqual(testCase,basis'*basis,eye(4),'AbsTol',1e-10);
verifyError(testCase,@() make_laguerre_basis(10,4,1), ...
    'MATLAB:notLess');
end

function testTrackingBaselines(testCase)
for controller = {'ltv','laguerre','nonlinear'}
    result = run_mpc_demo(controller{1},'tracking',Duration=3,Save=false);
    verifyTrue(testCase,result.metrics.success);
    verifyLessThan(testCase,result.metrics.track_cte_rms,0.02);
    verifyLessThan(testCase,result.metrics.speed_rmse,0.05);
    verifyLessThanOrEqual(testCase,max(abs(diff([0;result.logs.delta]))), ...
        result.parameters.delta_rate_max*result.parameters.T+1e-5);
end
end

function testLaguerreCoefficientsAreTrueIncrements(testCase)
p = parameters(); track = generate_track(p);
path = plan_reference_path(track,make_demo_obstacles(track,p,'tracking',15,21),p);
x = [path.X(1);path.Y(1);path.psi(1);p.v_tgt];
previous = [0;0];
reference = make_reference(path,x,p);
obstacles = make_demo_obstacles(track,p,'tracking',15,21);
[sequence,diagnostic] = mpc_solve(x,previous,reference,track,obstacles,p, ...
    'laguerre',repmat(previous,1,p.Nc));
basis = make_laguerre_basis(p.Nc,p.laguerre_order,p.laguerre_alpha);
expected = reshape(kron(basis,eye(2))*diagnostic.eta,2,p.Nc);
verifyGreaterThan(testCase,diagnostic.exitflag,0);
verifyEqual(testCase,diff([previous,sequence],1,2),expected,'AbsTol',1e-10);
end
