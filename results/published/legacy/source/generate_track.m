function track = generate_track(p)
% Parametric oval track: X = a*cos(t), Y = b*sin(t), t in [0, 2*pi).
%
% Heading is the tangent direction of motion:
%   psi = atan2(dY/dt, dX/dt) = atan2(b*cos(t), -a*sin(t))

t = linspace(0, 2*pi, p.N_wp + 1);
t = t(1:end-1);   % drop duplicate endpoint to keep the loop closed

track.X   = p.track_a * cos(t);
track.Y   = p.track_b * sin(t);
track.psi = atan2(p.track_b * cos(t), -p.track_a * sin(t));
track.N   = p.N_wp;
xy = [track.X(:), track.Y(:)];
track.s = [0; cumsum(vecnorm(diff(xy), 2, 2))];
track.length = track.s(end) + norm(xy(end,:) - xy(1,:));
end
