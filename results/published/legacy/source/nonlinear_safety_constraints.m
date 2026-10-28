function inequalities = nonlinear_safety_constraints(X,holder,p)
inequalities = zeros(p.Np*(height(holder.obstacles)+1),1);
index = 0;
for i = 1:p.Np
    pose = X(i+1,:)';
    center = pose(1:2)+p.body_offset*[cos(pose(3));sin(pose(3))];
    for j = 1:height(holder.obstacles)
        index = index+1;
        inequalities(index) = holder.obstacles.R(j)+p.body_radius+p.clearance+0.01- ...
            norm(center-[holder.obstacles.X(j);holder.obstacles.Y(j)]);
    end
    [~,distance] = project_path(holder.track,center);
    index = index+1;
    inequalities(index) = distance+p.body_radius+p.clearance-p.road_half_width;
end
end
