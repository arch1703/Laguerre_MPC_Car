function obstacles = generate_static_obstacles(track, numObstacles, minSpacingWp, radius, rngSeed)
% Generate static obstacles placed on the track centerline with spacing.

if nargin < 5
    rngSeed = 11;
end

rng(rngSeed);
N = track.N;
chosen = zeros(numObstacles, 1);
count = 0;
tries = 0;
maxTries = 5000;

while count < numObstacles && tries < maxTries
    tries = tries + 1;
    idx = randi(N);
    ok = true;
    for i = 1:count
        d = abs(idx - chosen(i));
        dCirc = min(d, N - d);
        if dCirc < minSpacingWp
            ok = false;
            break;
        end
    end
    if ok
        count = count + 1;
        chosen(count) = idx;
    end
end

if count < numObstacles
    error('Could not place all obstacles. Reduce numObstacles or minSpacingWp.');
end

chosen = sort(chosen);
X = track.X(chosen(:));
Y = track.Y(chosen(:));
side = sign(randn(numObstacles, 1));
side(side == 0) = 1;

obstacles = table(chosen(:), X(:), Y(:), repmat(radius, numObstacles, 1), side(:), ...
    'VariableNames', {'idx', 'X', 'Y', 'R', 'side'});
end
