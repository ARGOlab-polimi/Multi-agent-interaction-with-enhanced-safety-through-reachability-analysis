function [limiti, closest_point_idx] = check_limits(rel_states, g)


% Check if each element is within the bounds
check = (rel_states >= g.min') & (rel_states <= g.max');


% Checl if every state is inside the grid
if all(check)
    limiti = 1;
    closest_point_idx = [];
else
    limiti = 0;

    do_whole_approximated_routine = false;
    if do_whole_approximated_routine
        % Reshape the grid matrices into a 2D matrix where each row is a point in 6D space
        grid_points = [g.xs{1}(:), g.xs{2}(:), g.xs{3}(:), g.xs{4}(:), g.xs{5}(:), g.xs{6}(:)];

        % Compute the Euclidean distance between each grid point and the vector
        distances = sqrt(sum((grid_points - rel_states).^2, 2));

        % Retrieve the closest point index in the grid
        [~, closest_point_idx] = min(distances);

    else

        % cp = closest point
        cp_indeces = zeros(1, 6);

        % Loop over each dimension and find the closest value in the vector
        for i = 1:6
            % Find the absolute differences between the target point's value in this dimension
            % and all values in the corresponding vector g.vs{i}
            [~, idx] = min(abs(g.vs{i} - rel_states(i))/g.dx(i));

            % Store the closest value in the result array
            cp_indeces(i) = idx;
        end

        % Convert the 6 indices to a single linear index
        closest_point_idx = sub2ind(size(g.xs{1}), cp_indeces(1), cp_indeces(2), cp_indeces(3), ...
            cp_indeces(4), cp_indeces(5), cp_indeces(6));


    end
end