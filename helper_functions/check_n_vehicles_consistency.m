function check_n_vehicles_consistency(h, num_vehicles)


% Count non-empty elements in the cell array
nonEmptyCount = sum(~cellfun(@isempty, h));

% Check if non-empty count matches expected count
if nonEmptyCount ~= num_vehicles
    error('The number of inserted vehicles (%d) does not match the expected one (%d).', ...
        nonEmptyCount, num_vehicles);
end