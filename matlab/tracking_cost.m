function cost = tracking_cost(X,U,holder,p)
error = X(2:end,:)-holder.ref;
error(:,3) = atan2(sin(error(:,3)),cos(error(:,3)));
changes = diff([holder.previous';U(1:p.Nc,:)],1,1);
cost = sum(sum(error.^2.*p.Q_))+sum(sum(changes.^2.*p.R_));
end
