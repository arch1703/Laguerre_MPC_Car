function [safe,clearance] = check_control_sequence(x,previous,U,obstacles,track,p)
safe = true;
clearance = Inf;
for i = 1:p.Np
    u = U(:,min(i,size(U,2)));
    rates = abs(u-previous);
    if any(u < [-p.delta_max;p.a_min]-p.solver_tolerance) || ...
            any(u > [p.delta_max;p.a_max]+p.solver_tolerance) || ...
            any(rates > [p.delta_rate_max;p.jerk_max]*p.T+p.solver_tolerance)
        safe = false;
        return
    end
    [valid,gap] = check_motion(x,u,obstacles,track,p);
    clearance = min(clearance,gap);
    if ~valid
        safe = false;
        return
    end
    x = bicycle_step(x,u,p);
    previous = u;
end
end
