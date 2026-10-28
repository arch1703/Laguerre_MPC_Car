function directory = save_mpc_run(result,root)
if ~exist(root,'dir')
    mkdir(root);
end
[~,suffix] = fileparts(tempname(root));
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
directory = fullfile(root,[stamp,'_', ...
    result.controller,'_',result.scenario,'_',suffix]);
mkdir(directory);
result.run_dir = directory;
writetable(result.logs,fullfile(directory,'results.csv'));
track_table = table(result.track.X(:),result.track.Y(:),result.track.psi(:), ...
    'VariableNames',{'X','Y','psi'});
if isfield(result.track,'left')
    track_table.left_X = result.track.left(:,1); track_table.left_Y = result.track.left(:,2);
    track_table.right_X = result.track.right(:,1); track_table.right_Y = result.track.right(:,2);
end
writetable(track_table,fullfile(directory,'track.csv'));
writetable(result.obstacles,fullfile(directory,'obstacles.csv'));
if isfield(result,'obstacle_history')
    writetable(result.obstacle_history,fullfile(directory,'obstacle_history.csv'));
end
if isfield(result,'line_history')
    writetable(result.line_history,fullfile(directory,'line_history.csv'));
end
if isfield(result,'measurement_history')
    writetable(result.measurement_history,fullfile(directory,'measurement_history.csv'));
end
if ~isempty(result.events)
    writetable(struct2table(result.events),fullfile(directory,'events.csv'));
end
path = result.path;
writetable(table(path.X(:),path.Y(:),path.psi(:),path.v(:), ...
    'VariableNames',{'X','Y','psi','v'}),fullfile(directory,'planned_path.csv'));
writetable(array2table(result.final_state','VariableNames',{'X','Y','psi','v'}), ...
    fullfile(directory,'terminal_state.csv'));
source_root = fileparts(mfilename('fullpath'));
source_dir = fullfile(directory,'source');
mkdir(source_dir);
sources = dir(fullfile(source_root,'*.m'));
hashes = struct();
for i = 1:numel(sources)
    filename = fullfile(source_root,sources(i).name);
    digest = java.security.MessageDigest.getInstance('SHA-256');
    digest.update(unicode2native(fileread(filename),'UTF-8'));
    bytes = typecast(digest.digest(),'uint8');
    hashes.(matlab.lang.makeValidName(sources(i).name)) = lower(reshape(dec2hex(bytes,2)',1,[]));
    copyfile(filename,fullfile(source_dir,sources(i).name));
end
manifest = struct('schema_version',1,'created_at',char(datetime('now','TimeZone','UTC')), ...
    'controller',result.controller,'scenario',result.scenario,'seed',result.seed, ...
    'parameters',result.parameters,'metrics',result.metrics,'matlab_version',version, ...
    'toolboxes',ver,'source_sha256',hashes,'initial_state',result.initial_state, ...
    'requested_duration',result.requested_duration,'events',result.events, ...
    'spawn_policy','Generated exposures wait for separated locations ahead; replay uses saved positions/times');
if isfield(result,'world')
    manifest.world = result.world;
    manifest.spawn_policy = result.world.spawn_policy;
end
file = fopen(fullfile(directory,'manifest.json'),'w');
if file < 0
    error('mpc:SaveFailed','Cannot create run manifest in %s.',directory);
end
cleanup = onCleanup(@() fclose(file));
fprintf(file,'%s\n',jsonencode(manifest,'PrettyPrint',true));
save(fullfile(directory,'run.mat'),'result');
end
