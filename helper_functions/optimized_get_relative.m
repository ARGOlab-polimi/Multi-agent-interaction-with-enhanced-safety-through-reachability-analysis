function [rel_states, rel_dyn] = optimized_get_relative(ego_xy, u0, simX_0, simU_0, model)

    lr = model.params.lr;
    lf = model.params.lf;
    inv_lr_lf = 1 / (lr+lf);  % Precompute reciprocal

    cos_theta = cos(ego_xy(3));
    sin_theta = sin(ego_xy(3));

    % Relative transformation (direct computation)
    dx = simX_0(1) - ego_xy(1);
    dy = simX_0(2) - ego_xy(2);
    dtheta = simX_0(3) - ego_xy(3);

    x_rel = cos_theta * dx + sin_theta * dy;
    y_rel = -sin_theta * dx + cos_theta * dy;

    % Relative states
    rel_states = [x_rel, y_rel, dtheta, simX_0(4), ego_xy(4), ego_xy(5)];

    % Precompute beta_e and common_term
    tan_delta = tan(rel_states(5));
    beta_e = atan((lr * inv_lr_lf) * tan_delta);
    cos_beta_e = cos(beta_e);
    sin_beta_e = sin(beta_e);
    common_term = (rel_states(6) * cos_beta_e * inv_lr_lf) * tan_delta;

    % Relative dynamics
    rel_dyn = [
        rel_states(4) * cos(rel_states(3)) - rel_states(6) * cos_beta_e + rel_states(2) * common_term;
        rel_states(4) * sin(rel_states(3)) - rel_states(6) * sin_beta_e - rel_states(1) * common_term;
        simU_0(1) - common_term;
        simU_0(2);
        u0(1);
        u0(2)
    ]';

end
