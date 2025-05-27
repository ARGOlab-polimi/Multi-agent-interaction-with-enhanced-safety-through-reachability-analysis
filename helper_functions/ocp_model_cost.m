function ocp_model = ocp_model_cost(ocp_model, model,Q,R,Qe,scale_cost,dt)

% Set cost type
ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_type_e', 'linear_ls');

nx = length(model.x);
nu = length(model.u);

% number of outputs is the concatenation of x and u
ny = nx + nu;
ny_e = nx; % number of terminal outputs

% The linear cost contributions is defined through Vx, Vu and Vz
Vx = zeros(ny, nx);
Vx_e = zeros(ny_e, nx);
Vu = zeros(ny, nu);

Vx(1:nx,:) = eye(nx);
Vx_e(1:nx,:) = eye(nx);
Vu(nx+1:end,:) = eye(nu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_Vu', Vu);

% Define cost on states and input
if scale_cost
    W = blkdiag(Q, R)/dt;
    W_e = Qe*dt;
else
    W = blkdiag(Q, R);
    W_e = Qe;
end
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);

% set intial references
y_ref = [ model.x_ref, 0, 0];
y_ref_e = model.x_ref;
ocp_model.set('cost_y_ref', y_ref'); % it will updated afterwards
ocp_model.set('cost_y_ref_e', y_ref_e');