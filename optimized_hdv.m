classdef optimized_hdv
    properties
        simX                                                                % hdv states history
        simU                                                                % hdv control history
        simX_0                                                              % hdv current state
        simU_0                                                              % hdv current control
        rel_states                                                          % hdv relative states wrt ego
        rel_dyn                                                             % hdv relative dynamics wrt ego
        V                                                                   % Value function of hdv
        x                                                                   % grid vector limits
        grad                                                                % gridded gradient matrices of value function
        V_matrix                                                            % gridded value function
        p                                                                   % gradient of value function
        params                                                              % acados solver parameters vector
        g                                                                   % BRT grid
        settings                                                            % hdv collection of settings
        constr                                                              % hdv collection of constraints
        closest_point_idx                                                   % closest out of grid point
        limiti                                                              % inside grid flag vector
    end

    methods
        function obj = optimized_hdv(x0, BRT_filedata, control_flag, wh_coeff, ...
                Nsim, V_thr, h_constr)
            % Initialize vehicle state and disturbance model
            obj.simX = zeros(Nsim, length(x0));                             % Initializing states
            obj.simU = zeros(Nsim, 2);                                      % Initializing inputs
            obj.simX_0 = x0;                                                % x, y, theta, v
            obj.simU_0 = [0, 0];                                            % acc, w

            load(BRT_filedata);                                             % Load BRT struct
            obj.g = BRT.g;                                                  % assign BRT grid
            obj.x = BRT.g.vs;                                               % assign grid limits (vectors)
            obj.grad = BRT.grad;                                            % assign gradients
            obj.V_matrix = BRT.data;                                        % assign value function

            clear BRT

            obj.V = zeros(Nsim, 1);                                         % Initialize value function
            obj.p = zeros(Nsim, 6);                                         % Initialize gradients

            obj.limiti = zeros(Nsim,1);                                     % Initialize limits flag

            obj.rel_states = zeros(Nsim,6);                                 % Initialize relative states
            obj.rel_dyn = zeros(Nsim,6);                                    % Initialize relative dynamics

            obj.params = zeros(1,7);                                        % Initialize vector solver parameters

            % Initialize settings (struct to store parameters)
            obj.settings.p_threshold = 1e-2;                                % Optimal disturbance gradient threshold
            obj.settings.do_hdv_control = control_flag;                     % Compute disturbance flag
            obj.settings.wh_coeff = wh_coeff;                               % hdv yaw rate penalty term
            obj.settings.V_threshold = V_thr;                               % Value function activation threshold
            obj.settings.attiva_BRT = false;                                % Initial value of BRT constraint condition
            obj.settings.add_penalty = false;                               % Additional disturbance penalty flag
            obj.settings.onoff = zeros(Nsim,1);                             % BRT activation flag
            obj.settings.disabled = zeros(Nsim,1);                          % not dangerous BRT flag

            % Initialize constraints (struct to store constraint flags)
            obj.constr.add_circ_dist_constr = h_constr.add_circ_dist_constr;
            obj.constr.add_eucl_dist = h_constr.add_eucl_dist;
            obj.constr.add_left_over = h_constr.add_left_over;
            obj.constr.add_BRT = h_constr.add_BRT;

        end

        function obj = computeRelative( obj,ego_xy, u0, model,i)
            % Compute relative states and relative dynamics
            [obj.rel_states(i,:), obj.rel_dyn(i,:)] = optimized_get_relative(ego_xy, u0, obj.simX_0, obj.simU_0, model);
        end
       
        function obj = checkLimitsAndInterpolate(obj, i)
            % Check if current relative states is inside grid limits,
            % otherwise returns the closest point of the grid
            [obj.limiti(i), obj.closest_point_idx] = optimized_check_limits(obj.rel_states(i,:), obj.g);

            % Interpolate of the current point or extract the exact value 
            % of the out of grid point 
            obj = optimized_interpolate_V_grad(obj, i);
        end

        function obj = updateBRT(obj, i, lane_width)
            % Activation of BRT constraint check
            obj = optimized_activate_BRT(obj, i, lane_width);
        end

        function obj = updateParameters(obj,u0,i)
            % Create solver parameters vector
            obj.params = optimized_set_params(obj,u0,i);
        end


        function obj = humanSimulate(obj,i, dt)
            % Compute optimal disturbance

            % Integrate human states due to optimal disturbance
            obj = human_integrate(obj,i,dt);
        end

    end % methods

end % class
