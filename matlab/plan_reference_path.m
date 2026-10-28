function path = plan_reference_path(track,obstacles,p)
offset = zeros(track.N,1);
weights = zeros(track.N,height(obstacles));
amplitudes = zeros(height(obstacles),1);
for j = 1:height(obstacles)
    [obstacle_s,~,center,heading] = project_path(track,[obstacles.X(j),obstacles.Y(j)]);
    distance = mod(track.s-obstacle_s+track.length/2,track.length)-track.length/2;
    q = min(1,abs(distance)/p.detour_length);
    % A compact C2 bump has no position, tangent, or curvature jump at its ends.
    weights(:,j) = (1-q.^2).^3;
    outward = [sin(heading),-cos(heading)];
    lateral = ([obstacles.X(j),obstacles.Y(j)]-center)*outward';
    amplitudes(j) = max(p.detour_offset, ...
        lateral+obstacles.R(j)+p.body_radius+p.clearance+0.10);
end
if ~isempty(amplitudes)
    maximum = max(amplitudes);
    offset = maximum*(1-prod(1-weights.*(amplitudes'/maximum),2));
end
path = track;
path.X = track.X(:)+offset.*sin(track.psi(:));
path.Y = track.Y(:)-offset.*cos(track.psi(:));
xy = [path.X,path.Y];
tangent = circshift(xy,-1)-circshift(xy,1);
path.psi = atan2(tangent(:,2),tangent(:,1));
ds = vecnorm(circshift(xy,-1)-xy,2,2);
path.s = [0;cumsum(ds(1:end-1))];
path.length = sum(ds);
angles = atan2(sin(circshift(path.psi,-1)-circshift(path.psi,1)), ...
    cos(circshift(path.psi,-1)-circshift(path.psi,1)));
path.curvature = angles./(ds+circshift(ds,1));
if max(abs(path.curvature)) > tan(p.delta_max)/p.L
    error('mpc:InfeasiblePath','Detour curvature exceeds the vehicle steering limit.');
end
path.v = min(p.v_tgt,sqrt(p.lateral_accel_max./max(abs(path.curvature),1e-8)));
for pass = 1:4
    for i = 1:track.N
        j = mod(i,track.N)+1;
        path.v(j) = min(path.v(j),sqrt(path.v(i)^2+2*p.a_max*ds(i)));
    end
    for i = track.N:-1:1
        j = mod(i,track.N)+1;
        path.v(i) = min(path.v(i),sqrt(path.v(j)^2-2*p.a_min*ds(i)));
    end
end
for i = 1:path.N
    pose = [path.X(i);path.Y(i);path.psi(i);path.v(i)];
    [clearance,road] = vehicle_clearance(pose,obstacles,track,p);
    if clearance < p.clearance || road < p.clearance
        error('mpc:InfeasiblePath','Planned footprint violates obstacle or road clearance.');
    end
end
end
