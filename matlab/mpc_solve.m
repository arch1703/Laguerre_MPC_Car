function [sequence, diagnostic] = mpc_solve(x, previous, reference, track, obstacles, p, controller, warm)
nu = 2;
[D,map,Q,R,bounds,lower,rates,options] = solver_workspace(p,controller);
rate_offset = [-previous; zeros((p.Nc-1)*nu,1)];
if strcmp(controller,'laguerre')
    base = repmat(previous,p.Nc,1);
else
    base = zeros(p.Nc*nu,1);
end
sequence = warm;
diagnostic = struct('exitflag',-1,'iterations',0,'variables',size(map,2), ...
    'constraint_violation',Inf,'eta',zeros(size(map,2),1), ...
    'model_position_error',0,'model_heading_error',0,'linearization_passes',0);
[road_stations,~,road_centers,road_headings] = project_path(track,reference(:,1:2));
road_normals = [-sin(road_headings),cos(road_headings)];
road_widths = repmat(p.road_half_width-p.body_radius-p.clearance,p.Np,1);
if isfield(track,'left')
    road_widths = interp1(track.s,track.half_width,road_stations)-p.body_radius-p.clearance;
end
base_rows = 4*p.Nc*nu;
total_rows = base_rows+4*p.Np+p.Np*height(obstacles);
Ai = zeros(total_rows,size(map,2));
bi = zeros(total_rows,1);
Ai(1:base_rows,:) = [map;-map;D*map;-D*map];
bi(1:base_rows) = [bounds-base;base-lower;rates-D*base-rate_offset; ...
    rates+D*base+rate_offset];
iterations = 3;
if p.linearization_guard, iterations = p.linearization_iterations; end
for iteration = 1:iterations
    diagnostic.linearization_passes = iteration;
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
    for i = 1:p.Np
        rows = (i-1)*4+(1:4);
        speed_row = response(rows(4),:);
        center = road_centers(i,:);
        normal = road_normals(i,:);
        if isfield(track,'left')
            psi = reference(i,3);
            jacobian = [1,0,-p.body_offset*sin(psi),0; ...
                        0,1,p.body_offset*cos(psi),0];
            body_free = jacobian*free(rows)+p.body_offset*[cos(psi);sin(psi)]- ...
                jacobian(:,3)*psi;
            road_response = normal*jacobian*response(rows,:);
            road_free = normal*(body_free-center');
        else
            road_response = normal*response(rows(1:2),:);
            road_free = normal*(free(rows(1:2))-center');
        end
        width = road_widths(i);
        indices = base_rows+(i-1)*4+(1:4);
        Ai(indices,:) = [speed_row;-speed_row;road_response;-road_response];
        bi(indices) = [p.v_max-free(rows(4));free(rows(4))-p.v_min; ...
            width-road_free;width+road_free];
    end

    [Ao,bo] = obstacle_halfspaces(free,response,reference,obstacles,p);
    obstacle_rows = base_rows+4*p.Np+(1:size(Ao,1));
    Ai(obstacle_rows,:) = Ao;
    bi(obstacle_rows) = bo;
    constraints = Ai; limits = bi;
    if p.linearization_guard
        trust = repmat([p.linearization_steering_trust; ...
            p.linearization_acceleration_trust],p.Nc,1);
        nominal_inputs = sequence(:);
        constraints = [Ai;map;-map];
        limits = [bi;nominal_inputs-base+trust;base-nominal_inputs+trust];
    end
    [coefficients,~,flag,output] = quadprog(H,f,constraints,limits,[],[],[],[],[],options);
    diagnostic.exitflag = flag;
    diagnostic.iterations = diagnostic.iterations+output.iterations;
    if flag <= 0
        return
    end
    diagnostic.constraint_violation = max([0;constraints*coefficients-limits]);
    diagnostic.eta = coefficients;
    updated = reshape(base+map*coefficients,nu,p.Nc);
    accurate = true;
    if p.linearization_guard
        predicted = reshape(free+response*coefficients,4,p.Np);
        state = x; actual = zeros(4,p.Np);
        for i = 1:p.Np
            state = bicycle_step(state,updated(:,min(i,p.Nc)),p);
            actual(:,i) = state;
        end
        diagnostic.model_position_error = max(vecnorm(actual(1:2,:)-predicted(1:2,:),2,1));
        angle_error = actual(3,:)-predicted(3,:);
        diagnostic.model_heading_error = max(abs(atan2(sin(angle_error),cos(angle_error))));
        accurate = diagnostic.model_position_error<=p.linearization_position_tolerance && ...
            diagnostic.model_heading_error<=p.linearization_heading_tolerance;
    end
    converged = max(abs(updated-sequence),[],'all')<1e-4;
    sequence = updated;
    if converged && accurate
        break
    end
end
if p.linearization_guard && ~accurate
    diagnostic.exitflag = -20;
end
end

function [D,map,Q,R,bounds,lower,rates,options] = solver_workspace(p,controller)
persistent key cached
current = [controller,jsonencode(p)];
if isempty(key) || ~strcmp(key,current)
    D = kron(eye(p.Nc)-diag(ones(p.Nc-1,1),-1),eye(2));
    if strcmp(controller,'laguerre')
        basis = make_laguerre_basis(p.Nc,p.laguerre_order,p.laguerre_alpha);
        map = kron(tril(ones(p.Nc))*basis,eye(2));
    else
        map = eye(p.Nc*2);
    end
    Q = kron(eye(p.Np),diag(p.Q_)); R = kron(eye(p.Nc),diag(p.R_));
    bounds = repmat([p.delta_max;p.a_max],p.Nc,1);
    lower = repmat([-p.delta_max;p.a_min],p.Nc,1);
    rates = repmat([p.delta_rate_max;p.jerk_max]*p.T,p.Nc,1);
    options = optimoptions('quadprog','Display','off', ...
        'ConstraintTolerance',p.solver_tolerance,'OptimalityTolerance',p.solver_tolerance, ...
        'MaxIterations',p.qp_max_iterations);
    cached = {D,map,Q,R,bounds,lower,rates,options};
    key = current;
else
    [D,map,Q,R,bounds,lower,rates,options] = cached{:};
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
