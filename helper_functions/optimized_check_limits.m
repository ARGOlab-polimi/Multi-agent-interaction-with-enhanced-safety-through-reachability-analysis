function [limiti, closest_point_idx] = optimized_check_limits(rel_states, g)


% Check if each element is within the bounds
check = (rel_states >= g.min') & (rel_states <= g.max');


% Checl if every state is inside the grid
if all(check)
    limiti = 1;
    closest_point_idx = [];
else
    limiti = 0;

        % Convert the 6 indices to a single linear index
        closest_point_idx = 1;

    
end