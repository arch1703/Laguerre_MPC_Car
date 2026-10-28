function [sequence, diagnostic] = mpc_solve(x, previous, reference, track, obstacles, p, controller, warm)
nu = 2;
D = kron(eye(p.Nc)-diag(ones(p.Nc-1,1),-1),eye(nu));
rate_offset = [-previous; zeros((p.Nc-1)*nu,1)];
if strcmp(controller,'laguerre')
    basis = make_laguerre_basis(p.Nc,p.laguerre_order,p.laguerre_alpha);
    map = kron(tril(ones(p.Nc))*basis,eye(nu));
    base = repmat(previous,p.Nc,1);
else
    map = eye(p.Nc*nu);
    base = zeros(p.Nc*nu,1);
end
Q = kron(eye(p.Np),diag(p.Q_));
R = kron(eye(p.Nc),diag(p.R_));
bounds = repmat([p.delta_max; p.a_max],p.Nc,1);
lower = repmat([-p.delta_max; p.a_min],p.Nc,1);
rates = repmat([p.delta_rate_max; p.jerk_max]*p.T,p.Nc,1);
options = optimoptions('quadprog','Display','off','ConstraintTolerance', ...
    p.solver_tolerance,'OptimalityTolerance',p.solver_tolerance);
sequence = warm;
diagnostic = struct('exitflag',-1,'iterations',0,'variables',size(map,2), ...
    'constraint_violation',Inf,'eta',zeros(size(map,2),1));
[~,~,road_centers,road_headings] = project_path(track,reference(:,1:2));
road_normals = [-sin(road_headings),cos(road_headings)];
for iteration = 1:3
    A = zeros(4,4,p.Np);
    B = zeros(4,2,p.Np);
    c = zeros(4,p.Np);
    nominal = x;
    for i = 1:p.Np
        ui = sequence(:,min(i,p.Nc));
        [Ac,Bc] = get_linearized_model(nominal(3),nominal(4),ui(1),p.L);
        A(:,:,i) = eye(4)+p.T*Ac;
        B(:,:,i) = p.T*Bc;
        next = bicycle_step(nominal,ui,p);
        c(:,i) = next-A(:,:,i)*nominal-B(:,:,i)*ui;
        nominal = next;
    end
    [Phi,Theta,C] = build_prediction_matrices(A,B,c,p.Np,p.Nc);
    free = Phi*x+C+Theta*base;
    response = Theta*map;
    error = free-reshape(reference',[],1);
    H = 2*(response'*Q*response+map'*D'*R*D*map);
    H = (H+H')/2+1e-9*eye(size(map,2));
    f = 2*(response'*Q*error+map'*D'*R*(D*base+rate_offset));
    Ai = [map; -map; D*map; -D*map];
    bi = [bounds-base; base-lower; rates-D*base-rate_offset; ...
        rates+D*base+rate_offset];
    base_rows = size(Ai,1);
    Ai = [Ai;zeros(4*p.Np,size(map,2))];
    bi = [bi;zeros(4*p.Np,1)];
    for i = 1:p.Np
        rows = (i-1)*4+(1:4);
        speed_row = response(rows(4),:);
        center = road_centers(i,:);
        normal = road_normals(i,:);
        road_response = normal*response(rows(1:2),:);
        road_free = normal*(free(rows(1:2))-center');
        width = p.road_half_width-p.body_radius-p.clearance;
        indices = base_rows+(i-1)*4+(1:4);
        Ai(indices,:) = [speed_row;-speed_row;road_response;-road_response];
        bi(indices) = [p.v_max-free(rows(4));free(rows(4))-p.v_min; ...
            width-road_free;width+road_free];
    end
    [Ao,bo] = obstacle_halfspaces(free,response,reference,obstacles,p);
    Ai = [Ai; Ao];
    bi = [bi; bo];
    [coefficients,~,flag,output] = quadprog(H,f,Ai,bi,[],[],[],[],[],options);
    diagnostic.exitflag = flag;
    diagnostic.iterations = diagnostic.iterations+output.iterations;
    if flag <= 0
        return
    end
    diagnostic.constraint_violation = max([0; Ai*coefficients-bi]);
    diagnostic.eta = coefficients;
    updated = reshape(base+map*coefficients,nu,p.Nc);
    converged = max(abs(updated-sequence),[],'all')<1e-4;
    sequence = updated;
    if converged
        break
    end
end
end

function [A,b] = obstacle_halfspaces(free,response,reference,obstacles,p)
A = zeros(p.Np*height(obstacles),size(response,2));
b = zeros(p.Np*height(obstacles),1);
for i = 1:p.Np
    rows = (i-1)*4+(1:4);
    psi = reference(i,3);
    offset = p.body_offset*[cos(psi);sin(psi)];
    jacobian = [1,0,-p.body_offset*sin(psi),0; ...
                0,1, p.body_offset*cos(psi),0];
    center_free = jacobian*free(rows)+offset-jacobian(:,3)*psi;
    center_response = jacobian*response(rows,:);
    center_ref = reference(i,1:2)'+offset;
    for j = 1:height(obstacles)
        obstacle = [obstacles.X(j);obstacles.Y(j)];
        separation = center_ref-obstacle;
        normal = separation/max(norm(separation),eps);
        radius = obstacles.R(j)+p.body_radius+p.clearance+0.01;
        index = (i-1)*height(obstacles)+j;
        A(index,:) = -normal'*center_response;
        b(index) = normal'*(center_free-obstacle)-radius;
    end
end
end
