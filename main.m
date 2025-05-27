clear all
close all

import casadi.*
addpath('helper_functions', 'tracks');

track_file = 'rettilineo.txt';
beginning = tic;

%% Horizon parameters
N = 150; % Number of prediction steps
T = 3;                                                                      % Time horizon length [s]
dt = T / N; % Time step length [s]
Tf = 15.00;                                                                 % Maximum simulation time [s]
Nsim = round(Tf / dt);                                                      % Number of simulations

%% Initialize hdvs

lane_width = 4; % width of a single lane

% Filedata to load the BRT
BRT_short = 'BRT data/BRT_22.mat';                                          % String for smaller BRT (same lane)
BRT_long = 'BRT data/BRTt7_flipped2.mat';                                   % String for bigger BRT (opp lane)

% Initial condition for hdv (dubin)
num_vehicles = 2;                                                           % Number of human vehicles
h = cell(1, num_vehicles);                                                  % Initialize hdv vehicle cell 

h1_x0 = [20, -lane_width/2, 0, 5];                                          % Hdv initial condition
do_h1_control = false;                                                      % Hdv control flag
wh1_coeff = 0.25;                                                           % Hdv yaw rate penalty (related to BRT computation)
V1_threshold = 0.7;                                                         % V threshold
h1_constr.add_circ_dist_constr = false; % constraint flags
h1_constr.add_eucl_dist = false;
h1_constr.add_left_over = true;
h1_constr.add_BRT = true;
h{1} = hdv(h1_x0, BRT_short, do_h1_control, wh1_coeff, ...
    Nsim, V1_threshold, h1_constr);                                         % Initialize hdv

h2_x0 = [400, lane_width/2, pi, 5];                                         % Hdv initial condition
do_h2_control = false;                                                      % Hdv control flag
wh2_coeff = 0.01;                                                           % Hdv yaw rate penalty
V2_threshold = 1;                                                           % V threshold
h2_constr.add_circ_dist_constr = false;
h2_constr.add_eucl_dist = false;
h2_constr.add_left_over = false;
h2_constr.add_BRT = true;
h{2} = hdv(h2_x0, BRT_long, do_h2_control, wh2_coeff, ...
    Nsim, V2_threshold, h2_constr);                                         % Initialize hdv


%% Store only useful data

% List of variables you want to keep
vars_to_keep = {'track_file', 'beginning', 'N', 'T', 'dt', 'Tf', 'Nsim', ...
    'BRT_short', 'BRT_long', 'num_vehicles', 'h', 'lane_width'}; 

% Clear all variables except those specified 
clearvars('-except',vars_to_keep{:})

[h_x0, V_thresholds, h_constr] = store_hdv_params(h);

%% Internal check to avoid acados crash

check_n_vehicles_consistency(h, num_vehicles);

%% acados ocp model
% Model variables, dynamics, parameters and constraints
add_ego_soft = [false true true];
[model, constraint] = bicycle_model(h, lane_width, add_ego_soft);

% Model object
ocp_model = acados_ocp_model();

ocp_model.set('name', model.name);
ocp_model.set('T', T);

% Symbolics
ocp_model.set('sym_x', model.x);                                            % State
ocp_model.set('sym_u', model.u);                                            % Control
ocp_model.set('sym_xdot', model.xdot);                                      % State_dot
ocp_model.set('sym_p', model.p);                                            % Parameters

% Set constraints
% Soft constraints weights 
l_l1 = 1*[1 1];
l_l2 = [2000 2000];
u_l1 = 0.00001*[20 20 20 40 40 100];
u_l2 = 1*[3 3 10 0.050 0.100 50];

%%% How to read weights:
%%% in "l_" weights the first element is referred to the BRT weight of
%%% vehicles in the same lane, the second to vehicles in the other lane
%%% 
%%% in "u_" the first three elements are the weights for left/right road
%%% boundary and the high velocities promotion. the 4th accounts for the
%%% distance weights wrt cars in the same lane, the 5th the same for the
%%% other lane. the 6th accounts for the left overtake constraint weight

ocp_model = ocp_model_constraints(ocp_model, model, constraint, ...         % Set constraints
    l_l1, l_l2, u_l1, u_l2, h, add_ego_soft);

% Set cost
Q = diag([50, 1, 5e1, 1e-8, 1e-8]);                                         % State: s, y, theta, delta, v
R = diag([1e-2, 1e-8]);                                                     % Input: derDelta, derV
Qe = Q / 2;                                                                 % Terminal state

scale_cost = false;
ocp_model = ocp_model_cost(ocp_model, model, Q, R, Qe, scale_cost, dt);     % Set cost

% Set settings
[ocp_opts, settings] = ocp_solver_settings(ocp_model, model, N);

%% Create ocp solver
ocp_solver = acados_ocp(ocp_model, ocp_opts);

%% Simulate

% Initialize data structs
simX = zeros(Nsim, length(model.x));                                        % Initialize states
simU = zeros(Nsim, length(model.u));                                        % Initialize inputs

ocp_solver.set('constr_x0', model.x0);                                      % Define initial condition
ocp_solver.set('constr_lbx', model.x0, 0);                                  % Define lower constraint for IC at step 0
ocp_solver.set('constr_ubx', model.x0, 0);                                  % Define upper constraint for IC at step 0

% Set trajectory initialization
ocp_solver.set('init_x', model.x0' * ones(1, N + 1));                       % Set trajectory initialization (states)
ocp_solver.set('init_u', zeros(length(model.u), N));                        % Set trajectory initialization (inputs)
ocp_solver.set('init_pi', zeros(length(model.x), N));                       % Lagrange multipliers initialization

x0_ego = model.x0;                                                          % Ego initial condition
u0 = [0; 0];                                                                % Assume no control at the start of simulation

% Initialize slack and cost
SL = zeros(Nsim, ocp_solver.ocp.dims.nsh);                                  % Initialize lower slacks
SU = zeros(Nsim, ocp_solver.ocp.dims.nsh);                                  % Initialize upper slacks
cost_f = zeros(Nsim, 1);                                                    % Initialize cost vector
loop_times = zeros(Nsim, 1);                                                % Initialize solver time vector
short_horizon = 3;                                                          % BRT constraint short horizon 
change_reference = true;                                                    % Enable attentive level logic

% Simulate
for i = 1:Nsim

    tic

    % Transform ego state to X-Y coordinate reference system
    ego_xy = x0_ego;                                                        % No transformation in a straight road

    for v = 1:num_vehicles
        h{v} = h{v}.computeRelative(ego_xy, u0, model, i);                  % Get relative states and dynamics
        h{v} = h{v}.checkLimitsAndInterpolate(i);                           % Check if inside grid limits & interpolate
        h{v} = h{v}.updateBRT(i, v, lane_width);                            % Set BRT constraint
        h{v} = h{v}.updateParameters(u0, i);                                % Update parameters
    end

    ocp_solver = setSolverParameters(ocp_solver, num_vehicles, h, ...
        N, short_horizon, dt);                                              % Set parameters for solver
    ocp_solver = setEgoReference(ocp_solver, model, x0_ego, N, T, h, ...
        i, change_reference);                                               % Update ego reference for solver

    ocp_solver.solve();                                                     % Solve ocp

    u0 = ocp_solver.get('u', 0);                                            % Get control input

    simX(i, :) = ocp_solver.get('x', 0);                                    % Append to time history (states)
    simU(i, :) = u0;                                                        % Append to time history (inputs)
  
    x0_ego = ocp_solver.get('x', 1);                                        % Update ego initial condition
    ocp_solver.set('constr_x0', x0_ego);                                    % Set new initial condition
    ocp_solver.set('constr_lbx', x0_ego, 0);                                % Update lower bound constraint
    ocp_solver.set('constr_ubx', x0_ego, 0);                                % Update upper bound constraint

    SL(i,:) = ocp_solver.get('sl', 1);                                      % Get slack variables (lower)
    SU(i,:) = ocp_solver.get('su', 1);                                      % Get slack variables (upper)

    for v = 1:num_vehicles
        h{v} = h{v}.computeHamiltonian(u0, i);                              % Get Hamiltonian value
        h{v} = h{v}.humanSimulate(i, model, dt);                            % Simulate HDV dynamics
        h{v} = h{v}.getDistances(x0_ego, model, i);                         % Get distances between ego and HDV
    end

    cost_f(i) = ocp_solver.get_cost();                                      % Append the cost
    iter_time = toc;
    loop_times(i) = iter_time;                                              % Append iteration time
end

%% Plots
t = linspace(0.0, Nsim * dt, Nsim);                                         % Create time vector


% Plot track progression
traj_setting.do_animate = true;
traj_setting.anim_rate = 0.01 * dt;                                         % Animation rate
traj_setting.do_video = false;                                              % Video flag
traj_setting.do_BRT_plot = false;                                           % BRT plotting flag


plot_trajectories(t, simX, h, track_file, model, traj_setting, ...
     BRT_short, BRT_long, lane_width,[],[]);    

plot_full_trajectories(t, simX, h, track_file, false, lane_width);          % Plot time-colored evolution

% Plot value function and limits
for v = 1:num_vehicles
    V_H_4_thesis(t, h{v}.V, h{v}.limiti, h{v}.U_safe, ...
        h{v}.settings.V_threshold, v, h{v}.settings.onoff,0,22);
end


