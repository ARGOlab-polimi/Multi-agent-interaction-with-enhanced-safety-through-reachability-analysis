function [model, constraint] = bicycle_model(h, lane_width, add_ego_soft)

import casadi.*
model = struct();
constraint = struct();

model_name = 'Spatialbicycle_model';

%% CasADi Model Setup

% Set up ego states
s = SX.sym('s');                                                            % position progression
y = SX.sym('y');                                                            % centerline deviation
theta = SX.sym('theta');                                                    % orientation relative to the path
delta = SX.sym('delta');                                                    % steering angle
v = SX.sym('v');                                                            % velocity
x = vertcat(s, y, theta, delta, v);                                         % state vector

% Set up ego controls
derDelta = SX.sym('derDelta');                                              % change rate of steering angle
derV = SX.sym('derV');                                                      % velocity change rate
u = vertcat(derDelta, derV);                                                % input vector

% Set up xdot
sdot = SX.sym('sdot');
ydot = SX.sym('ydot');
thetadot = SX.sym('thetadot');
deltadot = SX.sym('deltadot');
vdot = SX.sym('vdot');
xdot = vertcat(sdot, ydot, thetadot, deltadot, vdot);                       % x_dot

%% Problem Parameters

% Ego vehicle dimensions
D1 = 4.68;
D2 = 2.2;
D = sqrt((D1/2)^2 + (D2/2)^2);
alpha = asin(D2/D1);

% HDV vehicle dimensions
D1_h = 4.28;
D2_h = 1.8;
D_h = sqrt((D1_h/2)^2 + (D2_h/2)^2);

% Lane width
y_lim = lane_width;

% Vehicle parameters for dynamics
lf = 0.959;
lr = 1.706;

%% Define HDV Parameters Symbolically
num_vehicles = length(h);

% List of parameters for each vehicle depending on constraints to add
n_params = 0;
for i = 1:num_vehicles

    % Initialize params_per_vehicle based on add_BRT constraint
    if h{i}.constr.add_BRT
        params_per_vehicle = {'p5', 'p6', 'bhj', 'onoff'};
    else
        params_per_vehicle = {};
    end

    % Add parameters based on additional constraints
    if h{i}.constr.add_circ_dist_constr
        params_per_vehicle = [params_per_vehicle, {'shdv', 'yhdv', 'thdv'}];
    elseif h{i}.constr.add_eucl_dist || h{i}.constr.add_left_over           % both constraints use same parameters
        params_per_vehicle = [params_per_vehicle, {'shdv', 'yhdv'}];
    end

    % Update maximum number of parameters if current list is longer
    n_params = max(n_params, length(params_per_vehicle));

    % Store the list in the vehicle constraints
    h{i}.constr.params_per_vehicle = params_per_vehicle;
end
    
hdv_params = cell(num_vehicles, n_params);                                  % initialize cell
for i = 1:num_vehicles
    for j = 1:length(h{i}.constr.params_per_vehicle)
        param_name = sprintf('h%d_%s', i, h{i}.constr.params_per_vehicle{j}); % create parameter string
        hdv_params{i, j} = SX.sym(param_name);                              % assign symbolic variable
    end
end

%% Helper Function for Distance Computation
    function dist = compute_distance(D1, D2, D1_h, D2_h, s, y, theta, shdv, yhdv, thetahdv, signs)
        dist = ((s + signs(1)*D1/4 * cos(theta) - shdv - signs(2)*D1_h/4 * cos(thetahdv))^2 + ...
            (y + signs(1)*D1/4 * sin(theta) - yhdv - signs(2)*D1_h/4 * sin(thetahdv))^2 - ...
            (sqrt((D1/4)^2 + (D2/2)^2) + sqrt((D1_h/4)^2 + (D2_h/2)^2))^2);
    end

%% Compute Distances for Vehicles
distances = cell(num_vehicles*4,1);                                         % initialize distance cell for each vehicle
sign_convention = [1,1;                                                     % create signs combinations
    1,-1;
    -1,1;
    -1,-1];



for i = 1:num_vehicles
    if h{i}.constr.add_circ_dist_constr
        % Current hdv states
        shdv = hdv_params{i, 5};
        yhdv = hdv_params{i, 6};
        thetahdv = hdv_params{i, 7};

        for j = 1:size(sign_convention,1)

            % Assign distance to cell
            distances{4*(i-1)+j} = compute_distance(D1, D2, D1_h, D2_h, ...
                s, y, theta, shdv, yhdv, thetahdv, sign_convention(j,:));
        end
    
    elseif h{i}.constr.add_eucl_dist
        % Current hdv states
        shdv = hdv_params{i, 5};
        yhdv = hdv_params{i, 6};

        distances{4*(i-1)+1} = ((s-shdv)^2+(y-yhdv)^2-(D+D_h)^2);
    end
end

%% Constraints and Bounds

% Lane boundary constraints
y_left = y + D * sin(alpha + abs(theta)) - (y_lim);
y_right = y - D * sin(alpha + abs(theta)) - (-y_lim);

% Define only ego soft constraints
k_lane_balance = (-3*y_lim/2 + D*sin(alpha))/(-y_lim/2 + D*sin(alpha));
k_lane = 3;
k_v = 10;
C_left = exp(k_lane * (y_left));
C_right = exp(k_lane * k_lane_balance * (-y_right));
C_v = exp(-k_v * (v - 1));

if any(add_ego_soft)
    C_ego_tmp = vertcat(C_left, C_right, C_v);
    idx_tmp = 1:3;
    C_ego = C_ego_tmp(idx_tmp(add_ego_soft));
else
    C_ego = [];
end

% Soft constraints for vehicles
C_hdv_dist = [];
k_dist = 1;
for i = 1:num_vehicles
    for j = 1:4
        C_hdv_dist = vertcat(C_hdv_dist, exp(-k_dist*distances{4*(i-1)+j}));       % Append distance soft constraint
    end
end

% Overtaking constraint (apply only if vehicle is in same direction)
C_overtake = [];
k_overtake = 0.05; 
for i = 1:num_vehicles
    if h{i}.constr.add_left_over

        ang_h_i = mod(h{i}.simX_0(3) + pi, 2*pi) - pi;                      % Hdv angle

        if ang_h_i >= -pi/2 && ang_h_i <= pi/2                              % If same direction
            right_overtake = (y - hdv_params{i, 6} + k_overtake * (s - hdv_params{i, 5})^2); % Adjust k as needed
            C_overtake = vertcat(C_overtake, exp(-right_overtake));             % Append overtake soft constraint
        end
    end
end

% Coefficient vector for soft constraints
k_ego = [k_lane, k_lane*k_lane_balance, k_v];
k_C_vec = [k_ego(add_ego_soft), ...
    k_dist*ones(1, length(C_hdv_dist)), k_overtake*ones(1,length(C_overtake))]; 

% Collect all soft constraints
C_vec = vertcat(C_ego, C_hdv_dist, C_overtake);

% Define control safety constraints
u_safe = [];
for i = 1:num_vehicles
    if h{i}.constr.add_BRT
        u_safe = vertcat(u_safe, ...                                            % Append BRT constraint for each hdv
            hdv_params{i, 4} * (hdv_params{i, 1} * derDelta + hdv_params{i, 2} * derV + hdv_params{i, 3}));
    end
end

%% Dynamics
beta = atan(lr / (lr + lf) * tan(delta));
f_expl = vertcat(...
    v * cos(theta + beta), ...
    v * sin(theta + beta), ...
    v * cos(beta) / (lr + lf) * tan(delta), ...
    derDelta, ...
    derV);

% Set up model bounds
model.y_min = -y_lim+D2/2;
model.y_max = y_lim-D2/2;
model.v_min = 1.0;
model.v_max = 11.0;
model.delta_min = -0.2618;
model.delta_max = 0.2618;

% Input bounds
model.ddelta_min = -0.087;
model.ddelta_max = 0.087;
model.dv_min = -7.0;
model.dv_max = 2.5;

% Initial conditions
model.v_ref = 8;
model.x0 = [0, -y_lim/2, 0, 0, model.v_ref];
model.x_ref = [0, -y_lim/2, 0, 0, model.v_ref];

%% Collect expressions for constraints and dynamics

constraint.x_expr = vertcat(y, v, delta);
constraint.u_expr = vertcat(derDelta, derV);
constraint.nonli_expr = vertcat(C_vec, u_safe);
constraint.expr = vertcat(constraint.x_expr, constraint.u_expr, constraint.nonli_expr);

hdv_params = hdv_params';

% Define model dynamics and parameters
model.f_impl_expr = f_expl - xdot;
model.f_expl_expr = f_expl;
model.x = x;
model.xdot = xdot;
model.u = u;
model.z = [];
model.p = vertcat(hdv_params{:});
model.name = model_name;
model.params.lf = lf;
model.params.lr = lr;
model.params.D1 = D1;
model.params.D2 = D2;
model.params.D = D;
model.params.D_h = D_h;
model.params.D1_h = D1_h;
model.params.D2_h = D2_h;
model.params.k_C_vec = k_C_vec;

end
