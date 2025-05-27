function [h_x0, V_thresholds, h_constr] = store_hdv_params(h)

% Initialize the structs to store all the h vehicle data
h_x0 = struct();         % Stores initial conditions for each vehicle
V_thresholds = struct(); % Stores V thresholds for each vehicle
h_constr = struct();     % Stores constraint booleans for each vehicle

% Loop over each vehicle stored in h cell array
for i = 1:numel(h)
    % Retrieve initial conditions from h{i}
    h_x0_field = sprintf('h%d_x0', i);
    h_x0.(h_x0_field) = h{i}.simX_0;  % Assuming x0 is stored as a property in h{i}

    % Retrieve V threshold from h{i}
    V_threshold_field = sprintf('V%d_threshold', i);
    V_thresholds.(V_threshold_field) = h{i}.settings.V_threshold;  % Assuming V_threshold is stored as a property in h{i}

    % Retrieve and store constraints from h{i}.constr
    constr_field = sprintf('h%d_constr', i);
    constr_struct = h{i}.constr;  % Assuming constr is a struct in h{i}
    
    % Extract booleans from the constr struct
    boolean_vector = struct2array(constr_struct);
    h_constr.(constr_field) = double(boolean_vector);
end