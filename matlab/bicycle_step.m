function next = bicycle_step(x, u, p, dt)
if nargin < 4
    dt = p.T;
end
next = x(:) + dt * [x(4)*cos(x(3)); x(4)*sin(x(3)); ...
    x(4)*tan(u(1))/p.L; u(2)];
end
