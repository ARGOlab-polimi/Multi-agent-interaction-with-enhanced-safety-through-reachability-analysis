function ocp_solver = optimized_setEgoReference(ocp_solver,model,x0_ego,N,T,h,i,change_reference)

%%% creare un controllo per quando sto gia chiudendo il sorpasso, allora la
%%% cosa piu semplice e accelerare se il veicolo viene da dietro, invece e
%%% frenare se viene da davanti

% Counter for inside limit occurrences
limitCount = 0;

% Iterate through the cell array
for ii = 1:length(h)
    % if h{ii}.('settings').('onoff')(i) == 1 || h{ii}.('settings').('disabled')(i) == 1
    if h{ii}.limiti(i) == 1 && h{ii}.V(i) < h{ii}.settings.V_threshold + h{ii}.simX_0(4)/2.5 ...
            + abs(x0_ego(5) - h{ii}.simX_0(4)*cos(h{ii}.simX_0(3))) *(h{ii}.simX_0(2)>0)
        
% it was 2
        limitCount = limitCount + 1;
    end

end

yref = [model.x0, 0, 0];


% Emergency maneuver
emergency_merge = false;
emergency_follow = false; 


if change_reference && limitCount >= 2
    if ego_ontheright(h{1}.simX_0(1:2), h{2}.simX_0(1:2), x0_ego(1:2)) ...
            || x0_ego(1)>h{1}.simX_0(1)+model.params.D1/2+model.params.D1_h/2
        emergency_merge = true;

    elseif x0_ego(3) > 0.02 %x0_ego(2) > 0 ||
        emergency_follow = true;

    end
end


if limitCount < 2 || ~change_reference 
    for j = 0:N-1
        yref(1) = x0_ego(1)+model.v_ref*j/N*T;
        ocp_solver.set('cost_y_ref', yref, j); % define reference at step j
    end

    yref_N = model.x0;
    yref_N(1) = x0_ego(1)+model.v_ref*T;
    ocp_solver.set('cost_y_ref_e', yref_N); % set new terminal reference

elseif emergency_merge 

    for j = 0:N-1
        yref(1) = x0_ego(1)+model.v_max*j/N*T;
        yref(end) = model.dv_max;
        ocp_solver.set('cost_y_ref', yref, j); % define reference at step j
    end

    yref_N = model.x0;
    yref_N(1) = x0_ego(1)+model.v_max*T;
    ocp_solver.set('cost_y_ref_e', yref_N); % set new terminal reference

elseif emergency_follow


    d_safe = 4+model.params.D1/2+model.params.D1_h/2; % safe distance

    % Reference velocity for automerge and hdv1 following
    v_ref = (h{1}.simX_0(1)+h{1}.simX_0(4)*T-d_safe-x0_ego(1))/T; 


    for j = 0:N-1
        yref(1) = x0_ego(1)+v_ref*j/N*T;
        yref(end-1) = model.ddelta_min;
        yref(end) = model.dv_min;
        ocp_solver.set('cost_y_ref', yref, j); % define reference at step j
    end

    yref_N = model.x0;
    yref_N(1) = x0_ego(1)+v_ref*T;
    ocp_solver.set('cost_y_ref_e', yref_N); % set new terminal reference


else 

    
    d_safe = 4+model.params.D1/2+model.params.D1_h/2; % safe distance

    % Reference velocity for automerge and hdv1 following
    v_ref = (h{1}.simX_0(1)+h{1}.simX_0(4)*T-d_safe-x0_ego(1))/T; 
    
    for j = 0:N-1
        yref(1) = x0_ego(1)+v_ref*j/N*T;
        yref(end) = model.dv_min;
        yref(5) = h{1}.simX_0(4);
        ocp_solver.set('cost_y_ref', yref, j); % define reference at step j
    end

    yref_N = model.x0;
    yref_N(1) = x0_ego(1)+v_ref*T;
    yref_N(5) = h{1}.simX_0(4);
    ocp_solver.set('cost_y_ref_e', yref_N); % set new terminal reference


end

end

function  isright = ego_ontheright(A, B, P)


% A is the vehicle to overtake
% B is the vehicle the 2nd vehicle
% P is the ego

safe_offset = 2;

% Compute the line passing through A and B

m = (B(2) - A(2))/(B(1) - A(1));

q  = A(2) - m * (A(1) + safe_offset);

% Check if ego is above the hdv connecting line
if  P(1) > (P(2)-q)/m
    isright = true;
else
    isright = false;
end
end
