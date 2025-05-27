function h = activate_BRT(h,i,n_vehicle, lane_width)


if h.V(i) > h.settings.V_threshold || ~h.limiti(i)                          % if above threshold or outside grid limits

    onoff = eps;                                                            % de-activate BRT constraint

elseif (1.1*h.rel_states(i,2)-lane_width) >= 0  && check_theta_rel(h.rel_states(i,3), 5e-2) ...
        && h.simX_0(2) > 0 || (h.rel_states(i,2) < - 0 && h.simX_0(2) > 0)

    % If inside limits and below threshold but in a not dangerous situation
    h.settings.disabled(i) = true;
    onoff = eps;                                                            % de-activate BRT constraint

    % onoff = 1.0;
    disp([num2str(n_vehicle),' - ', num2str(i),': disabled'])

else

    onoff = 1.0;                                                            % activate BRT constraint

    disp([num2str(n_vehicle),' - ', num2str(i),': attivo'])

end

h.settings.onoff(i) = onoff;
end


function k = check_theta_rel(theta, epsilon)

    % Normalize theta within [0, 2*pi] for easier handling
    theta = mod(theta, 2*pi);
    
    % Check if theta is approximately equal to 0 or pi within epsilon
    if abs(theta - pi) <= epsilon || abs(theta) <= epsilon || abs(theta - 2*pi) <= epsilon
        k = true;  % Return true if theta is approximately 0 or pi
    else
        k = false; % Return false otherwise
    end

end

