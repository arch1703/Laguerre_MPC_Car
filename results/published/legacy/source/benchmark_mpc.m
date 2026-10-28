function summary = benchmark_mpc(profile,options)
arguments
    profile (1,:) char {mustBeMember(profile,{'quick','full'})} = 'quick'
    options.OutputRoot (1,:) char = fullfile(fileparts(mfilename('fullpath')),'..','results','benchmarks')
end
if ~exist(options.OutputRoot,'dir'), mkdir(options.OutputRoot); end
directory = tempname(options.OutputRoot);
mkdir(directory);
cases = struct('name',{},'controller',{},'scenario',{},'duration',{},'parameters',{},'seed',{});
for controller = {'laguerre','ltv','nonlinear'}
    cases(end+1) = definition([controller{1},'_tracking'],controller{1},'tracking',15,struct(),21); %#ok<AGROW>
end
for controller = {'laguerre','ltv'}
    cases(end+1) = definition([controller{1},'_static'],controller{1},'static',15,struct(),21); %#ok<AGROW>
end
cases(end+1) = definition('laguerre_appearing','laguerre','appearing',45,struct(),21);
cases(end+1) = definition('ltv_appearing_replay','ltv','appearing',45,struct(),21);
if strcmp(profile,'full')
    for order = [2,4,6]
        for alpha = [0.4,0.7,0.9]
            if order==4 && alpha==0.7, continue; end
            cases(end+1) = definition(sprintf('basis_%d_%.1f',order,alpha), ...
                'laguerre','static',15,struct('laguerre_order',order,'laguerre_alpha',alpha),21); %#ok<AGROW>
        end
    end
    for horizon = [10,30]
        cases(end+1) = definition(sprintf('horizon_%d',horizon),'laguerre','static',15, ...
            struct('Np',horizon,'Nc',horizon/2),21); %#ok<AGROW>
    end
    for speed = [0.8,1.6]
        cases(end+1) = definition(sprintf('speed_%.1f',speed),'laguerre','static',15, ...
            struct('v_tgt',speed),21); %#ok<AGROW>
    end
    for seed = [7,37]
        cases(end+1) = definition(sprintf('seed_%d',seed),'laguerre','static',15,struct(),seed); %#ok<AGROW>
    end
    for count = [1,4]
        cases(end+1) = definition(sprintf('obstacles_%d',count),'laguerre','static',15, ...
            struct('obstacle_count',count),21); %#ok<AGROW>
    end
end
rows = struct([]);
appearing = [];
for i = 1:numel(cases)
    test = cases(i);
    fprintf('Benchmark %d/%d: %s\n',i,numel(cases),test.name);
    started = tic;
    try
        if strcmp(test.name,'ltv_appearing_replay')
            result = replay_mpc_run(appearing.run_dir,'ltv',OutputRoot=fullfile(directory,'runs'));
        else
            result = run_mpc_demo(test.controller,test.scenario,Duration=test.duration, ...
                Parameters=test.parameters,Seed=test.seed,OutputRoot=fullfile(directory,'runs'));
        end
        if strcmp(test.name,'laguerre_appearing'), appearing = result; end
        metrics = result.metrics;
        p = result.parameters;
        row = struct('name',test.name,'controller',test.controller,'scenario',test.scenario, ...
            'success',metrics.success,'termination',metrics.termination, ...
            'path_error_rms_cm',100*metrics.path_error_rms, ...
            'track_cte_rms_cm',100*metrics.track_cte_rms,'speed_rmse',metrics.speed_rmse, ...
            'clearance_cm',100*metrics.min_clearance,'road_clearance_cm',100*metrics.min_road_clearance, ...
            'collision_samples',metrics.collision_samples,'fallback_steps',metrics.fallback_steps, ...
            'solver_failures',metrics.solver_failures,'laps',metrics.laps, ...
            'solve_median_ms',1000*metrics.solve_time_median, ...
            'solve_p95_ms',1000*metrics.solve_time_p95,'solve_p99_ms',1000*metrics.solve_time_p99, ...
            'step_p99_ms',1000*metrics.step_time_p99,'missed_deadlines',metrics.missed_step_deadlines, ...
            'variables',max(result.logs.variables),'Np',p.Np,'Nc',p.Nc, ...
            'order',p.laguerre_order,'alpha',p.laguerre_alpha,'target_speed',p.v_tgt, ...
            'seed',test.seed,'obstacle_count',height(result.obstacles),'run_directory',result.run_dir);
    catch exception
        if ~ismember(exception.identifier,{'mpc:InfeasiblePath', ...
                'mpc:InvalidRoad','mpc:NoSafeSpawn'})
            rethrow(exception);
        end
        warning('mpc:BenchmarkInfeasible','%s: %s',test.name,exception.message);
        if isempty(rows)
            rethrow(exception);
        end
        row = rows(1);
        fields = fieldnames(row);
        for j = 1:numel(fields)
            if isnumeric(row.(fields{j})), row.(fields{j}) = NaN; end
        end
        row.name = test.name; row.controller = test.controller; row.scenario = test.scenario;
        row.success = false; row.termination = exception.message; row.run_directory = '';
        row.seed = test.seed;
    end
    row.wall_time_s = toc(started);
    rows = [rows;row]; %#ok<AGROW>
    summary = struct2table(rows);
    writetable(summary,fullfile(directory,'summary.csv'));
    save(fullfile(directory,'benchmark.mat'),'cases','summary','profile');
end
fprintf('Benchmark results: %s\n',fullfile(directory,'summary.csv'));
disp(summary(:,{'name','success','path_error_rms_cm','clearance_cm','solve_median_ms','step_p99_ms'}));
end

function test = definition(name,controller,scenario,duration,parameters,seed)
test = struct('name',name,'controller',controller,'scenario',scenario, ...
    'duration',duration,'parameters',parameters,'seed',seed);
end
