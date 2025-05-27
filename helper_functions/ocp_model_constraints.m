function ocp_model = ocp_model_constraints(ocp_model, model, constraint, l_l1, l_l2, u_l1, u_l2, h, ...
    add_ego_soft)

nx = length(model.x);
nu = length(model.u);

% State constraint
nbx = length(constraint.x_expr);
Jbx = zeros(nbx,nx);
Jbx(1,2) = 1;                                                               % y
Jbx(2,4) = 1;                                                               % delta
Jbx(3,5) = 1;                                                               % v
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', [model.y_min, model.delta_min, model.v_min]);
ocp_model.set('constr_ubx', [model.y_max, model.delta_max, model.v_max]);

% Input constraint 
nbu = length(constraint.u_expr);
Jbu = zeros(nbu,nu);
Jbu(1,1) = 1;                                                               % ddelta
Jbu(2,2) = 1;                                                               % dv
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', [model.ddelta_min, model.dv_min]);
ocp_model.set('constr_ubu', [model.ddelta_max, model.dv_max]);


%% Initialize weights for constraints

if any(add_ego_soft)
    u_l1_weights = u_l1(add_ego_soft);
    u_l2_weights = u_l2(add_ego_soft);
    l_l1_weights = zeros(size(u_l1_weights));
    l_l2_weights = zeros(size(u_l2_weights));
else
    l_l1_weights = [];
    l_l2_weights = [];
    u_l1_weights = [];
    u_l2_weights = [];
end


%% Update weights based on constraints in h
for i = 1:length(h)
    sim_y_positive = h{i}.simX_0(2) >= 0;

    if h{i}.constr.add_circ_dist_constr
        % Add weights for circular distance constraint
        l_l1_weights = [l_l1_weights, zeros(1, 4)];
        l_l2_weights = [l_l2_weights, zeros(1, 4)];
        if sim_y_positive
            u_l1_weights = [u_l1_weights, u_l1(5) * ones(1, 4)];
            u_l2_weights = [u_l2_weights, u_l2(5) * ones(1, 4)];
        else
            u_l1_weights = [u_l1_weights, u_l1(4) * ones(1, 4)];
            u_l2_weights = [u_l2_weights, u_l2(4) * ones(1, 4)];
        end
    elseif h{i}.constr.add_eucl_dist
        % Add weights for Euclidean distance constraint
        l_l1_weights = [l_l1_weights, 0];
        l_l2_weights = [l_l2_weights, 0];
        if sim_y_positive
            u_l1_weights = [u_l1_weights, u_l1(5)];
            u_l2_weights = [u_l2_weights, u_l2(5)];
        else
            u_l1_weights = [u_l1_weights, u_l1(4)];
            u_l2_weights = [u_l2_weights, u_l2(4)];
        end
    end
end

%% Additional weight updates
for i = 1:length(h)
    if h{i}.constr.add_left_over
        % Add weights for left-over constraint
        l_l1_weights = [l_l1_weights, 0];
        l_l2_weights = [l_l2_weights, 0];
        u_l1_weights = [u_l1_weights, u_l1(6)];
        u_l2_weights = [u_l2_weights, u_l2(6)];
    end
end

for i = 1:length(h)
    if h{i}.constr.add_BRT
        % Add weights for backward reachable tube constraint
        u_l1_weights = [u_l1_weights, 0];
        u_l2_weights = [u_l2_weights, 0];
        if sim_y_positive
            l_l1_weights = [l_l1_weights, l_l1(2)];
            l_l2_weights = [l_l2_weights, l_l2(2)];
        else
            l_l1_weights = [l_l1_weights, l_l1(1)];
            l_l2_weights = [l_l2_weights, l_l2(1)];
        end
    end
end


% Set upper bound for nonlinear constraints (just the BRT ones are
% non-zeros)
upper_bound = (l_l2_weights > 0) * 100 + 1e-3;


% Nonlinear constraint
nh = length(constraint.nonli_expr);
ocp_model.set('constr_expr_h_0', constraint.nonli_expr);                     % set initial nonlinear constraint
ocp_model.set('constr_lh_0', zeros(1,nh));                                   % lower initial nonlinear constraint bound
ocp_model.set('constr_uh_0', upper_bound);                                   % upper initial nonlinear constraint bound
ocp_model.set('constr_expr_h', constraint.nonli_expr);                       % set nonlinear constraint
ocp_model.set('constr_lh', zeros(1,nh));                                     % lower nonlinear constraint bound
ocp_model.set('constr_uh', upper_bound);                                     % upper nonlinear constraint bound

% Configure constraint slack variables for nonlinear constraint h
nsh = nh;
Jsh = zeros(nh,nsh);
Jsh(:,:) = eye(nsh);
ocp_model.set('constr_Jsh_0', Jsh);
ocp_model.set('constr_Jsh', Jsh);


%% Set cost on slack
% L1 slack (linear term)
ocp_model.set('cost_zl_0', (l_l1_weights)');                                % lower slack cost
ocp_model.set('cost_zu_0', (u_l1_weights)');                                % upper slack cost
ocp_model.set('cost_zl', (l_l1_weights)');                                  % lower slack cost
ocp_model.set('cost_zu', (u_l1_weights)');                                  % upper slack cost

%L2 slack (squared term)
ocp_model.set('cost_Zl_0', diag(l_l2_weights));                             % lower slack cost
ocp_model.set('cost_Zu_0', diag(u_l2_weights));                             % upper slack cost
ocp_model.set('cost_Zl', diag(l_l2_weights));                               % lower slack cost
ocp_model.set('cost_Zu', diag(u_l2_weights));                               % upper slack cost

% Set intial condition
ocp_model.set('constr_x0', model.x0);
