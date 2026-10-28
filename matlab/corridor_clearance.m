function distance = corridor_clearance(corridor,position)
if corridor.closed
    left = boundary_path(corridor.left);
    right = boundary_path(corridor.right);
    [~,a] = project_path(left,position);
    [~,b] = project_path(right,position);
    distance = min(a,b);
    in_left = inpolygon(position(:,1),position(:,2),corridor.left(:,1),corridor.left(:,2));
    in_right = inpolygon(position(:,1),position(:,2),corridor.right(:,1),corridor.right(:,2));
    distance(~xor(in_left,in_right)) = -distance(~xor(in_left,in_right));
    return
end
polygon = [corridor.left;flipud(corridor.right)];
outline = boundary_path(polygon);
[~,distance] = project_path(outline,position);
inside = inpolygon(position(:,1),position(:,2),polygon(:,1),polygon(:,2));
distance(~inside) = -distance(~inside);
end

function path = boundary_path(points)
path = struct('X',points(:,1),'Y',points(:,2), ...
    's',[0;cumsum(vecnorm(diff(points),2,2))]);
end
