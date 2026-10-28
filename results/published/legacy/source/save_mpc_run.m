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
writetable(table(result.track.X(:),result.track.Y(:),result.track.psi(:), ...
    'VariableNames',{'X','Y','psi'}),fullfile(directory,'track.csv'));
writetable(result.obstacles,fullfile(directory,'obstacles.csv'));
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
file = fopen(fullfile(directory,'manifest.json'),'w');
if file < 0
    error('mpc:SaveFailed','Cannot create run manifest in %s.',directory);
end
cleanup = onCleanup(@() fclose(file));
fprintf(file,'%s\n',jsonencode(manifest,'PrettyPrint',true));
save(fullfile(directory,'run.mat'),'result');
end
