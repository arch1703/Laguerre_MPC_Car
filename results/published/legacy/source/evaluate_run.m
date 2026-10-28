function metrics = evaluate_run(logs,final,track,p)
metrics = struct();
metrics.duration = sum(logs.applied)*p.T;
metrics.track_cte_mean = mean(logs.track_cte);
metrics.track_cte_rms = sqrt(mean(logs.track_cte.^2));
metrics.track_cte_max = max(logs.track_cte);
metrics.path_error_rms = sqrt(mean(logs.path_error.^2));
metrics.speed_rmse = sqrt(mean((logs.v-logs.v_ref).^2));
metrics.mean_speed = mean(logs.v);
metrics.min_clearance = min(logs.min_clearance);
metrics.min_rectangle_clearance = min(logs.rectangle_clearance);
metrics.min_road_clearance = min(logs.road_clearance);
metrics.collision_samples = sum(logs.rectangle_clearance<0);
metrics.fallback_steps = sum(logs.fallback);
metrics.solver_failures = sum(logs.exitflag<=0 & logs.solver_called);
metrics.prediction_rejections = sum(~logs.prediction_valid & logs.solver_called);
metrics.solve_time_median = median(logs.solve_time);
metrics.solve_time_p95 = prctile(logs.solve_time,95);
metrics.solve_time_p99 = prctile(logs.solve_time,99);
metrics.missed_deadlines = sum(logs.solve_time>p.T);
metrics.step_time_p95 = prctile(logs.step_time,95);
metrics.step_time_p99 = prctile(logs.step_time,99);
metrics.missed_step_deadlines = sum(logs.step_time>p.T);
metrics.steering_rate_rms = sqrt(mean(diff([0;logs.delta]).^2))/p.T;
metrics.jerk_rms = sqrt(mean(diff([0;logs.a]).^2))/p.T;
xy = [logs.X,logs.Y;final(1:2)'];
metrics.path_length = sum(vecnorm(diff(xy),2,2));
progress = zeros(size(xy,1),1);
for i = 1:size(xy,1)
    progress(i) = project_path(track,xy(i,:));
end
delta = diff(progress);
delta = mod(delta+track.length/2,track.length)-track.length/2;
metrics.laps = sum(delta)/track.length;
metrics.success = all(logs.exitflag>0) && ...
    all(logs.min_clearance>=p.clearance-p.solver_tolerance) && ...
    all(logs.road_clearance>=p.clearance-p.solver_tolerance);
end
