function [safe,clearance,road,rectangle] = check_motion(x,u,obstacles,track,p)
next = bicycle_step(x,u,p);
poses = (x+(next-x)*((0:p.safety_substeps)/p.safety_substeps))';
[gap,edge,box_gap] = vehicle_clearance(poses,obstacles,track,p);
clearance = min(gap); road = min(edge); rectangle = min(box_gap);
% The body-center distance is Lipschitz; tighten sampled gaps between samples.
speed_bound = max(abs([x(4),next(4)]))+ ...
    abs(p.body_offset*x(4)*tan(u(1))/p.L);
buffer = speed_bound*p.T/(2*p.safety_substeps);
clearance = clearance-buffer;
road = road-buffer;
rectangle = rectangle-(max(abs([x(4),next(4)]))+ ...
    hypot(p.car_front,p.car_half_width)*abs(x(4)*tan(u(1))/p.L))* ...
    p.T/(2*p.safety_substeps);
safe = clearance >= p.clearance-p.solver_tolerance && ...
    road >= p.clearance-p.solver_tolerance && ...
    next(4) >= p.v_min-p.solver_tolerance && next(4) <= p.v_max+p.solver_tolerance;
end
