%% Example of the frc_racecars in simulation without obstacle avoidance:

clear all
close all

import casadi.*
addpath('helper_functions', 'tracks');

track_file = 'rettilineo.txt';

%% Horizon parameters
N = 150;
T = 3;                                                                      % Time horizon length
dt = T / N;
Tf = 20.00;                                                                 % Maximum simulation time [s]
Nsim = round(Tf / dt);                                                      % Number of simulations

%% Initialize hdvs

lane_width = 4;

% Filedata to load the BRT
BRT_short = '';                                          % String for smaller BRT
BRT_long = '';                                   % String for bigger in front BRT

% Initial condition for hdv (dubin)
num_vehicles = 2;                                                           % Number of vehicles
h = cell(1, num_vehicles);                                                  % Initialize hdv vehicle cell 

h1_x0 = [10, -lane_width/2, 0, 5];                                          % Hdv initial condition
do_h1_control = false;                                                      % Hdv control flag
wh1_coeff = 0.25;                                                           % Hdv yaw rate penalty
V1_threshold = 0.7;                                                         % V threshold
h1_constr.add_circ_dist_constr = false;
h1_constr.add_eucl_dist = false;
h1_constr.add_left_over = true;
h1_constr.add_BRT = true;
h{1} = optimized_hdv(h1_x0, BRT_short, do_h1_control, wh1_coeff, ...
    Nsim, V1_threshold, h1_constr);                                         % Initialize hdv

h2_x0 = [140, lane_width/2, pi, 11];                                         % Hdv initial condition
do_h2_control = false;                                                      % Hdv control flag
wh2_coeff = 0.01;                                                            % Hdv yaw rate penalty
V2_threshold = 1;                                                           % V threshold
h2_constr.add_circ_dist_constr = false;
h2_constr.add_eucl_dist = false;
h2_constr.add_left_over = false;
h2_constr.add_BRT = true;
h{2} = optimized_hdv(h2_x0, BRT_long, do_h2_control, wh2_coeff, ...
    Nsim, V2_threshold, h2_constr);                                         % Initialize hdv


%% Store only useful data

% List of variables you want to keep
vars_to_keep = {'track_file', 'beginning', 'N', 'T', 'dt', 'Tf', 'Nsim', ...
    'BRT_short', 'BRT_long', 'num_vehicles', 'h', 'lane_width'}; 

% Clear all variables except those specified
clearvars('-except',vars_to_keep{:})

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
l_l2 = 1*[2000 2000];
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
[ocp_opts, ~] = optimized_ocp_solver_settings(ocp_model, model, N);

%% Create ocp solver
ocp_solver = acados_ocp(ocp_model, ocp_opts);

clear constraint ocp_model Q R Qe l_l1 l_l2 u_l1 u_l2 add_ego_soft ocp_opts scale_cost 
%% Simulate

ocp_solver.set('constr_x0', model.x0);                                      % Define initial condition

% Set trajectory initialization
ocp_solver.set('init_x', model.x0' * ones(1, N + 1));                       % Set trajectory initialization (states)
ocp_solver.set('init_u', zeros(length(model.u), N));                        % Set trajectory initialization (inputs)
ocp_solver.set('init_pi', zeros(length(model.x), N));                       % Lagrange multipliers initialization

x0_ego = model.x0;                                                          % Ego initial condition
u0 = [0; 0];                                                                % Assume no control at the start of simulation

% Initialize slack and cost
loop_times = zeros(Nsim, 1);
short_horizon = 3;
change_reference = true;

% Simulate
for i = 1:Nsim
    tic

    for v = 1:num_vehicles
        h{v} = h{v}.computeRelative(x0_ego, u0, model, i);                  % Get relative states and dynamics
        h{v} = h{v}.checkLimitsAndInterpolate(i);                           % Check if inside grid limits & interpolate
        h{v} = h{v}.updateBRT(i, lane_width);                               % Set BRT constraint
        h{v} = h{v}.updateParameters(u0, i);                                % Update parameters
    end

    ocp_solver = optimized_setSolverParameters(ocp_solver, num_vehicles, h, ...
        N, short_horizon, dt);                                              % Set parameters for solver
    ocp_solver = optimized_setEgoReference(ocp_solver, model, x0_ego, N, T, h, ...
        i, change_reference);                                               % Update ego reference for solver

    ocp_solver.solve();                                                     % Solve ocp

    
    

    x0_ego = ocp_solver.get('x', 1);                                        % Update ego initial condition
    ocp_solver.set('constr_x0', x0_ego);                                    % Set new initial condition
  
    u0 = ocp_solver.get('u',0); 

    for v = 1:num_vehicles
        h{v} = h{v}.humanSimulate(i, dt);                            % Simulate HDV dynamics
    end

    iter_time = toc;

    loop_times(i) = iter_time;
end

%% Plots
t = linspace(0.0, Nsim * dt, Nsim);                                         % Create time vector

freq = 1./loop_times(2:end);
min_freq = min(freq)
max_freq = max(freq)
mean_freq = mean(freq)
figure;
subplot(121);plot(freq);yline(50);xlabel('Time step');title('Real time frequency');ylabel('Hz')
subplot(122);plot(t,loop_times*1000);yline(dt*1000);xlabel('Time [s]');title('Iteration time');ylabel('t [ms]')
freq_loss = sum(freq<(1/dt))


% Plot track progression
traj_setting.do_animate = true;
traj_setting.anim_rate = 0.1* dt;                                          % Animation rate
traj_setting.do_video = false;                                              % Video flag
traj_setting.do_BRT_plot = false;                                           % BRT plotting flag

plot_trajectories(t, simX, h, track_file, model, traj_setting, ...
    BRT_short, BRT_long, lane_width);                                       % Plot trajectories
