function result = replay_mpc_run(directory,controller,options)
arguments
    directory (1,:) char
    controller (1,:) char {mustBeMember(controller,{'laguerre','ltv','nonlinear'})} = 'laguerre'
    options.Save (1,1) logical = true
    options.Plot (1,1) logical = false
    options.OutputRoot (1,:) char = fullfile(fileparts(mfilename('fullpath')),'..','results','runs')
end
saved = load(fullfile(directory,'run.mat'),'result');
original = saved.result;
p = rmfield(original.parameters,{'body_offset','body_radius'});
result = run_mpc_demo(controller,original.scenario,Duration=original.requested_duration, ...
    Parameters=p,Seed=original.seed,InitialState=original.initial_state, ...
    Obstacles=original.obstacles,Save=options.Save,Plot=options.Plot,OutputRoot=options.OutputRoot);
end
