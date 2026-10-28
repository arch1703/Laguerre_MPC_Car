function fig = plot_results(result)
p = result.parameters;
logs = result.logs;
fig = figure('Name','MPC RC-car demonstration','Color','w');
tiledlayout(3,2);
nexttile([3,1]);
track = result.track;
plot(track.X,track.Y,'k--'); hold on;
for side = [-1,1]
    plot(track.X(:)-side*p.road_half_width*sin(track.psi(:)), ...
        track.Y(:)+side*p.road_half_width*cos(track.psi(:)),'Color',[0.7,0.7,0.7]);
end
plot(logs.X_ref,logs.Y_ref,'g-');
plot([logs.X;result.final_state(1)],[logs.Y;result.final_state(2)],'b-','LineWidth',1.5);
obs = result.obstacles;
obs = obs(obs.spawn_time<=result.metrics.duration,:);
theta = linspace(0,2*pi,100);
for i = 1:height(obs)
    plot(obs.X(i)+obs.R(i)*cos(theta),obs.Y(i)+obs.R(i)*sin(theta),'r-');
end
corners = [p.car_front,p.car_half_width;p.car_front,-p.car_half_width; ...
    -p.car_rear,-p.car_half_width;-p.car_rear,p.car_half_width];
pose = result.final_state;
rotation = [cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
body = corners*rotation'+pose(1:2)';
patch(body(:,1),body(:,2),'b','FaceAlpha',0.3);
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('%s / %s: %s',result.controller,result.scenario,result.metrics.termination));
nexttile(2);
plot(logs.t,100*logs.path_error,logs.t,100*logs.track_cte);
legend('Planned-path error','Track CTE','Location','best');
ylabel('Distance [cm]'); grid on;
nexttile(4);
yyaxis left;
plot(logs.t,logs.v); hold on;
plot(logs.t,logs.v_ref,'--');
ylabel('Speed [m/s]');
yyaxis right;
plot(logs.t,rad2deg(logs.delta));
yline(rad2deg(p.delta_max),'r--'); yline(-rad2deg(p.delta_max),'r--');
ylabel('Steering [deg]'); grid on;
nexttile(6);
plot(logs.t,logs.min_clearance,logs.t,logs.road_clearance);
yline(p.clearance,'r--','Required clearance');
legend('Conservative body/obstacle','Body/road','Location','best');
xlabel('Time [s]'); ylabel('Clearance [m]'); grid on;
end
