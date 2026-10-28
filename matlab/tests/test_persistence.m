function tests = test_persistence
tests = functiontests(localfunctions);
end

function testSelfContainedRunAndReplay(testCase)
directory = tempname;
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory,'s'));
original = run_mpc_demo('laguerre','tracking',Duration=0.5,OutputRoot=directory);
manifest = jsondecode(fileread(fullfile(original.run_dir,'manifest.json')));
verifyEqual(testCase,manifest.parameters.car_front,original.parameters.car_front);
verifyEqual(testCase,manifest.controller,'laguerre');
verifyEqual(testCase,height(readtable(fullfile(original.run_dir,'results.csv'))),10);
verifyTrue(testCase,isfile(fullfile(original.run_dir,'source','run_mpc_demo.m')));
verifyEqual(testCase,numel(manifest.source_sha256.run_mpc_demo_m),64);
replay = replay_mpc_run(original.run_dir,'laguerre',Save=false);
verifyEqual(testCase,replay.logs.X,original.logs.X,'AbsTol',1e-10);
verifyEqual(testCase,replay.logs.Y,original.logs.Y,'AbsTol',1e-10);
second = run_mpc_demo('ltv','tracking',Duration=0.1,OutputRoot=directory);
verifyNotEqual(testCase,second.run_dir,original.run_dir);
verifyTrue(testCase,isfile(fullfile(original.run_dir,'manifest.json')));
end

function testHeadlessPlot(testCase)
result = run_mpc_demo('laguerre','static',Duration=0.1,Save=false);
visibility = get(groot,'DefaultFigureVisible');
cleanup = onCleanup(@() set(groot,'DefaultFigureVisible',visibility));
set(groot,'DefaultFigureVisible','off');
fig = plot_results(result);
figure_cleanup = onCleanup(@() close(fig));
verifyTrue(testCase,isgraphics(fig));
end
