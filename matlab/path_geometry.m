function path = path_geometry(points,closed)
validateattributes(points,{'numeric'},{'2d','ncols',2,'finite','real'});
if size(points,1)<3 || any(vecnorm(diff(points),2,2)<1e-8)
    error('mpc:InvalidPath','Require at least three distinct consecutive points.');
end
ds = vecnorm(diff(points),2,2);
path = struct('X',points(:,1),'Y',points(:,2),'N',size(points,1), ...
    's',[0;cumsum(ds)],'length',sum(ds),'closed',closed);
if closed
    closing = norm(points(end,:)-points(1,:));
    if closing<1e-8, error('mpc:InvalidPath','Do not duplicate the closing point.'); end
    path.length = path.length+closing;
    tangent = circshift(points,-1)-circshift(points,1);
else
    tangent = [points(2,:)-points(1,:);points(3:end,:)-points(1:end-2,:); ...
        points(end,:)-points(end-1,:)];
end
path.psi = unwrap(atan2(tangent(:,2),tangent(:,1)));
path.curvature = gradient(path.psi)./gradient(path.s);
end
