function ocp_solver = optimized_setSolverParameters(ocp_solver, num_vehicles,h, N, short_horizon, dt)

p = cell(num_vehicles, 1); % total parameters
p_reduced = cell(num_vehicles, 1); % total reduced BRT parameters

integration_flags = [];

for i = 1:num_vehicles

    hdv_integrate = 0;

    % Initialize p{i} based on the main conditions (constraints
    if h{i}.constr.add_BRT
        if h{i}.constr.add_circ_dist_constr
            p{i} = h{i}.params;
            hdv_integrate = 3;
        elseif h{i}.constr.add_eucl_dist || h{i}.constr.add_left_over
            p{i} = h{i}.params(1:end-1);
            hdv_integrate = 2;
        else
            p{i} = h{i}.params(1:4);
             hdv_integrate = 0;
        end
    else
        if h{i}.constr.add_circ_dist_constr
            p{i} = h{i}.params(5:end);
            hdv_integrate = 3;
        elseif h{i}.constr.add_eucl_dist || h{i}.constr.add_left_over
            p{i} = h{i}.params(5:end-1);
        else
            p{i} = [];
        end

    end

    integration_flags = [integration_flags; zeros(size(p{i}))];

    if hdv_integrate
        hdv_dynamics = [h{i}.simX_0(4)*cos(h{i}.simX_0(3)); ... 
            h{i}.simX_0(4)*sin(h{i}.simX_0(3)); 0];
        integration_flags(end-hdv_integrate+1:end) = hdv_dynamics(1:hdv_integrate);
    end

    % Create p_reduced{i} based on p{i}
    p_reduced{i} = p{i};
    if h{i}.constr.add_BRT
        p_reduced{i}(4) = eps;  % Set the fourth element to epsilon if add_BRT is active
    end

end

p = vertcat(p{:}); % Concatenate horizontally to make p a row vector

p = p(:); % vector of all parameters

p_reduced = vertcat(p_reduced{:}); % Concatenate horizontally to make p a row vector

p_reduced = p_reduced(:); % vector of all parameters with disabled BRT coefficient


for j = 0:N

    if j < short_horizon
        % Time integration
        p_j = p+j*integration_flags*dt;

        ocp_solver.set('p', p_j, j);
    else
        % Time integration

        p_reduced_j = p_reduced+j*integration_flags*dt;
        ocp_solver.set('p', p_reduced_j, j);
    end

end


