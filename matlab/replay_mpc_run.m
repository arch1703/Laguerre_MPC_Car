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
if strcmp(original.scenario,'boundary_crossing')
    if strcmp(controller,'nonlinear')
        error('mpc:InvalidController','Boundary replay supports Laguerre and full-input LTV only.');
    end
    boundaries = struct('left',original.track.left,'right',original.track.right);
    experiment = struct();
    if isfield(original,'experiment'), experiment = original.experiment; end
    result = run_boundary_demo(original.world.track_kind,Duration=original.requested_duration, ...
        Boundaries=boundaries,Controller=controller,Experiment=experiment, ...
        Save=options.Save,Plot=options.Plot,OutputRoot=options.OutputRoot);
    return
end
p = rmfield(original.parameters,{'body_offset','body_radius'});
if strcmp(original.scenario,'moving')
    result = run_mpc_demo(controller,'moving',Duration=original.requested_duration, ...
        Parameters=p,InitialState=original.initial_state,Save=options.Save, ...
        Plot=options.Plot,OutputRoot=options.OutputRoot);
    return
end
if strcmp(original.scenario,'random')
    result = run_mpc_demo(controller,'random',Duration=original.requested_duration, ...
        Parameters=p,InitialState=original.initial_state,WorldSeed=original.world.seed, ...
        WorldParameters=original.world.parameters,Save=options.Save,Plot=options.Plot, ...
        OutputRoot=options.OutputRoot);
    return
end
result = run_mpc_demo(controller,original.scenario,Duration=original.requested_duration, ...
    Parameters=p,Seed=original.seed,InitialState=original.initial_state, ...
    Obstacles=original.obstacles,Save=options.Save,Plot=options.Plot,OutputRoot=options.OutputRoot);
end
