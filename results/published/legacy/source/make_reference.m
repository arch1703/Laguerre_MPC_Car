function [reference, current, progress] = make_reference(path, x, p)
[progress, ~, xy] = project_path(path, x(1:2));
s = [path.s(:); path.length];
heading = unwrap([path.psi(:); path.psi(1)]);
query = progress;
reference = zeros(p.Np, 4);
current_heading = interp1(s, heading, mod(query, path.length));
current = [xy, current_heading, interp1(s, [path.v(:); path.v(1)], ...
    mod(query, path.length))];
for i = 1:p.Np
    speed = interp1(s, [path.v(:); path.v(1)], mod(query, path.length));
    query = query + p.T*speed;
    wrapped = mod(query, path.length);
    reference(i,:) = [interp1(s, [path.X(:); path.X(1)], wrapped), ...
        interp1(s, [path.Y(:); path.Y(1)], wrapped), ...
        interp1(s, heading, wrapped), ...
        interp1(s, [path.v(:); path.v(1)], wrapped)];
end
angles = x(3) + unwrap([0; atan2(sin(reference(:,3)-x(3)), ...
    cos(reference(:,3)-x(3)))]);
reference(:,3) = angles(2:end);
end
