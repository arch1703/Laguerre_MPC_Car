function safe = check_sensor_visibility(x,sequence,p,sensing,offset)
if nargin<5, offset = 0; end
safe = true;
if ~sensing.complete_detection, return; end
% Absent objects can enter only from outside the last complete range scan.
for j = 0:size(sequence,2)
    if j>0, x = bicycle_step(x,sequence(:,j),p); end
    center = x(1:2)+p.body_offset*[cos(x(3));sin(x(3))];
    reserve = p.body_radius+p.clearance+sensing.unseen_radius_bound+ ...
        sensing.unseen_speed_bound*(sensing.range_age+offset+j*p.T);
    if norm(center-sensing.range_center)+reserve>sensing.range
        safe = false;
        return
    end
end
end
