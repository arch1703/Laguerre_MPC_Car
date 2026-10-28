function obstacles = make_demo_obstacles(track,p,scenario,duration,seed)
if strcmp(scenario,'tracking')
    count = 0;
else
    count = p.obstacle_count;
end
if count == 0
    obstacles = table(zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
        zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
        'VariableNames',{'id','idx','X','Y','R','side','spawn_step','spawn_time'});
    return
end
obstacles = generate_static_obstacles(track,count,round(0.20*track.N), ...
    p.obstacle_radius,seed);
obstacles.id = (1:count)';
obstacles.side(:) = -1;
obstacles.spawn_time = zeros(count,1);
if strcmp(scenario,'appearing')
    obstacles.spawn_time = linspace(min(0.8,duration/4),0.7*duration,count)';
    obstacles.X(:) = NaN;
    obstacles.Y(:) = NaN;
    obstacles.idx(:) = 0;
end
obstacles.spawn_step = round(obstacles.spawn_time/p.T)+1;
obstacles.requested_spawn_time = obstacles.spawn_time;
obstacles = movevars(obstacles,'id','Before',1);
end
