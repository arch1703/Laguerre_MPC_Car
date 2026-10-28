function [reference, current, progress] = make_reference(path, x, p)
[progress, ~, xy] = project_path(path, x(1:2));
closed = ~isfield(path,'closed') || path.closed;
if closed
    s = [path.s(:);path.length];
    headings = [path.psi(:);path.psi(1)];
    xs = [path.X(:);path.X(1)]; ys = [path.Y(:);path.Y(1)];
    speeds = [path.v(:);path.v(1)];
else
    s = path.s(:); headings = path.psi(:);
    xs = path.X(:); ys = path.Y(:); speeds = path.v(:);
end
heading = unwrap(headings);
query = progress;
reference = zeros(p.Np, 4);
wrapped = reference_query(query,path.length,closed);
current_heading = interp1(s,heading,wrapped);
current = [xy,current_heading,interp1(s,speeds,wrapped)];
for i = 1:p.Np
    speed = interp1(s,speeds,reference_query(query,path.length,closed));
    query = query + p.T*speed;
    wrapped = reference_query(query,path.length,closed);
    reference(i,:) = [interp1(s,xs,wrapped), ...
        interp1(s,ys,wrapped), ...
        interp1(s, heading, wrapped), ...
        interp1(s,speeds,wrapped)];
end

angles = x(3) + unwrap([0; atan2(sin(reference(:,3)-x(3)), ...
    cos(reference(:,3)-x(3)))]);
reference(:,3) = angles(2:end);
end

function value = reference_query(query,length,closed)
if closed, value = mod(query,length);
else, value = min(length,max(0,query));
end
end
