function [path,available] = plan_sensed_path(corridor,obstacles,x,p)
points = [corridor.X,corridor.Y];
normals = [-sin(corridor.psi),cos(corridor.psi)];
station = project_path(corridor,x(1:2));
best = Inf; available = false;
path = corridor;
path.v = zeros(corridor.N,1);
for side = [0,-1,1]
    weights = zeros(corridor.N,height(obstacles));
    for j = 1:height(obstacles)
        [s,~,center,heading] = project_path(corridor,[obstacles.X(j),obstacles.Y(j)]);
        lateral = ([obstacles.X(j),obstacles.Y(j)]-center)*[-sin(heading);cos(heading)];
        amplitude = max(0,obstacles.R(j)+p.body_radius+p.clearance+0.07+side*lateral);
        q = min(1,abs(corridor.s-s)/1.8);
        weights(:,j) = min(0.60,amplitude)*(1-q.^2).^3;
    end
    offset = zeros(corridor.N,1);
    if ~isempty(weights), offset = side*0.60*(1-prod(1-weights/0.60,2)); end
    candidate = path_geometry(points+offset.*normals,false);
    candidate.v = min(p.v_tgt,sqrt(p.lateral_accel_max./max(abs(candidate.curvature),1e-8)));
    if max(abs(candidate.curvature))>tan(p.delta_max)/p.L, continue; end
    ahead = corridor.s>=station-0.15 & corridor.s<=corridor.length-0.35;
    poses = [candidate.X(ahead),candidate.Y(ahead),candidate.psi(ahead),candidate.v(ahead)];
    [gap,edge] = vehicle_clearance(poses,obstacles,corridor,p);
    if any(gap<p.clearance) || any(edge<p.clearance), continue; end
    [~,distance] = project_path(candidate,x(1:2));
    cost = sum(offset.^2)+10*distance^2;
    if cost<best
        path = candidate; best = cost; available = true;
    end
end
end
