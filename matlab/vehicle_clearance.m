function [obstacle_gap,road_gap,rectangle_gap] = vehicle_clearance(x,obstacles,track,p)
if isvector(x), x = x(:)'; end
center = x(:,1:2)+p.body_offset*[cos(x(:,3)),sin(x(:,3))];
obstacle_gap = Inf(size(x,1),1);
rectangle_gap = obstacle_gap;
for j = 1:height(obstacles)
    obstacle = [obstacles.X(j),obstacles.Y(j)];
    obstacle_gap = min(obstacle_gap,vecnorm(center-obstacle,2,2)-p.body_radius-obstacles.R(j));
    delta = obstacle-x(:,1:2);
    relative = [cos(x(:,3)).*delta(:,1)+sin(x(:,3)).*delta(:,2), ...
        -sin(x(:,3)).*delta(:,1)+cos(x(:,3)).*delta(:,2)];
    q = abs(relative-[p.body_offset,0])- ...
        [(p.car_front+p.car_rear)/2,p.car_half_width];
    gap = vecnorm(max(q,0),2,2)+min(max(q,[],2),0)-obstacles.R(j);
    rectangle_gap = min(rectangle_gap,gap);
end
if isfield(track,'left')
    road_gap = corridor_clearance(track,center)-p.body_radius;
else
    [~,road_distance] = project_path(track,center);
    road_gap = p.road_half_width-p.body_radius-road_distance;
end
end
