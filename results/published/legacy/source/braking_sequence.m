function [sequence,safe] = braking_sequence(x,previous,obstacles,track,p)
N = p.Np;
speed_map = p.T*tril(ones(N));
D = eye(N)-diag(ones(N-1,1),-1);
offset = [-previous(2);zeros(N-1,1)];
H = 2*(speed_map'*speed_map+0.01*(D'*D));
f = 2*(speed_map'*(x(4)*ones(N,1))+0.01*D'*offset);
A = [speed_map;-speed_map;D;-D];
b = [(p.v_max-x(4))*ones(N,1);(x(4)-p.v_min)*ones(N,1); ...
    p.jerk_max*p.T*ones(N,1)-offset;p.jerk_max*p.T*ones(N,1)+offset];
opts = optimoptions('quadprog','Display','off');
[acceleration,~,flag] = quadprog(H,f,A,b,speed_map(end,:),-x(4), ...
    p.a_min*ones(N,1),p.a_max*ones(N,1),[],opts);
sequence = zeros(2,N);
safe = false;
if flag <= 0
    return
end
sequence(1,:) = previous(1);
sequence(2,:) = acceleration';
[safe,~] = check_control_sequence(x,previous,sequence,obstacles,track,p);
end
