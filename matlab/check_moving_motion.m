function [clearance,road,rectangle,contact] = check_moving_motion(x,u,world,time,track,p)
% Ground-truth monitor only: never supply its future samples to the controller.
next = bicycle_step(x,u,p);
clearance = Inf; road = Inf; rectangle = Inf; contact = false;
for i = 0:p.safety_substeps
    fraction = i/p.safety_substeps;
    pose = x+(next-x)*fraction;
    truth = world.snapshot(time+p.T*fraction);
    [gap,edge,box] = vehicle_clearance(pose,truth,track,p);
    clearance = min(clearance,gap);
    road = min(road,edge);
    rectangle = min(rectangle,box);
    contact = contact || box<=0;
end
speed = max(abs([x(4),next(4)]));
omega = abs(x(4)*tan(u(1))/p.L);
half_interval = p.T/(2*p.safety_substeps);
clearance = clearance-(speed+abs(p.body_offset)*omega+world.speed_bound())*half_interval;
road = road-(speed+abs(p.body_offset)*omega)*half_interval;
rectangle = rectangle-(speed+hypot(p.car_front,p.car_half_width)*omega+ ...
    world.speed_bound())*half_interval;
end
