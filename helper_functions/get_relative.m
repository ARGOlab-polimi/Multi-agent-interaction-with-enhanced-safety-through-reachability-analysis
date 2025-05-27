function [rel_states, rel_dyn] = get_relative(ego_xy, u0, simX_0, simU_0, model)

lr = model.params.lr;
lf = model.params.lf;

% Relative transformation matrix
xytheta_rel = [cos(ego_xy(3)) sin(ego_xy(3)) 0;
    -sin(ego_xy(3)) cos(ego_xy(3)) 0;
    0 0 1]...
    *[simX_0(1)-ego_xy(1); simX_0(2)-ego_xy(2); simX_0(3)-ego_xy(3)];

% Relative states
rel_states = [xytheta_rel; simX_0(4); ego_xy(4); ego_xy(5)]';


% Terms for relative dynamics
beta_e = atan((lr/(lr+lf))*tan(rel_states(5)));
common_term = (rel_states(6)*cos(beta_e)/(lr+lf))*tan(rel_states(5));

% Relative dynamics
rel_dyn = [
    rel_states(4)*cos(rel_states(3))-rel_states(6)*cos(beta_e)+rel_states(2)*common_term;
    rel_states(4)*sin(rel_states(3))-rel_states(6)*sin(beta_e)-rel_states(1)*common_term;
    simU_0(1)-common_term;
    simU_0(2);
    u0(1);
    u0(2)]';


