function [corridor,measurements] = sense_track_lines(truth,pose,range)
validateattributes(range,{'numeric'},{'scalar','finite','positive'});
station = project_path(truth,pose(1:2));
query = mod(station+(-1.0:0.10:range)',truth.length);
grid = [truth.s;truth.length];
left = interp1(grid,[truth.left;truth.left(1,:)],query);
right = interp1(grid,[truth.right;truth.right(1,:)],query);
valid = vecnorm(left-pose(1:2)',2,2)<=range & ...
    vecnorm(right-pose(1:2)',2,2)<=range;
indices = find(valid);
if numel(indices)<12 || any(diff(indices)~=1)
    error('mpc:MissingTrackLines','Insufficient contiguous local line measurements.');
end
left = left(valid,:); right = right(valid,:);
measurements = table(left(:,1),left(:,2),right(:,1),right(:,2), ...
    'VariableNames',{'left_X','left_Y','right_X','right_Y'});
corridor = path_geometry((left+right)/2,false);
corridor.left = left; corridor.right = right;
corridor.half_width = vecnorm(left-right,2,2)/2;
end
