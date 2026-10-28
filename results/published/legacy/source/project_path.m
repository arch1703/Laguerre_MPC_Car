function [s, distance, point, heading] = project_path(path, position)
xy = [path.X(:), path.Y(:)];
edge = circshift(xy, -1) - xy;
if isvector(position)
    position = position(:)';
end
dx = position(:,1)-xy(:,1)';
dy = position(:,2)-xy(:,2)';
lengths = sum(edge.^2,2);
fraction = (dx.*edge(:,1)'+dy.*edge(:,2)')./lengths';
fraction = max(0, min(1, fraction));
squared = (dx-fraction.*edge(:,1)').^2+(dy-fraction.*edge(:,2)').^2;
[distance,index] = min(squared,[],2);
distance = sqrt(distance);
indices = sub2ind(size(fraction),(1:size(position,1))',index);
weights = fraction(indices);
point = xy(index,:)+weights.*edge(index,:);
s = path.s(index)+weights.*sqrt(lengths(index));
heading = atan2(edge(index,2),edge(index,1));
end
