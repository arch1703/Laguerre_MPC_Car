function track = make_boundary_track(kind,p,boundaries)
if nargin<3, boundaries = struct(); end
if isempty(fieldnames(boundaries))
    theta = (0:p.N_wp-1)'*2*pi/p.N_wp;
    switch kind
        case 'oval'
            points = [4*cos(theta),2.8*sin(theta)];
        case 'kidney'
            points = [4*cos(theta)+0.65*cos(2*theta),3*sin(theta)];
        case 'chicane'
            points = [4.5*cos(theta),3*sin(theta)+0.65*sin(3*theta)];
        otherwise
            error('mpc:InvalidTrack','Track must be oval, kidney, chicane, or paired custom boundaries.');
    end
    track = path_geometry(points,true);
    normals = [-sin(track.psi),cos(track.psi)];
    track.left = points+p.road_half_width*normals;
    track.right = points-p.road_half_width*normals;
else
    if ~all(isfield(boundaries,{'left','right'}))
        error('mpc:InvalidTrack','Custom boundaries need paired left/right Nx2 points.');
    end
    validateattributes(boundaries.left,{'numeric'},{'2d','ncols',2,'finite','real'});
    validateattributes(boundaries.right,{'numeric'},{'size',size(boundaries.left),'finite','real'});
    track = path_geometry((boundaries.left+boundaries.right)/2,true);
    track.left = boundaries.left; track.right = boundaries.right;
end
track.half_width = vecnorm(track.left-track.right,2,2)/2;
normal = [-sin(track.psi),cos(track.psi)];
if any(sum((track.left-track.right).*normal,2)<=2*(p.body_radius+p.clearance))
    error('mpc:InvalidTrack','Boundaries must be ordered left/right with enough vehicle clearance.');
end
if max(abs(track.curvature))>tan(p.delta_max)/p.L
    error('mpc:InvalidTrack','Track centerline exceeds the vehicle turning limit.');
end
end
