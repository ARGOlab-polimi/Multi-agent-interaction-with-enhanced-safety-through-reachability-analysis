function [ocp_opts, settings] = optimized_ocp_solver_settings(ocp_model, model, N)

% To remove or insert a setting, just comment/uncomment it out in the
% solver parameters subsection

%% Solver parameters (see race_car for all options)
compile_interface = 'auto';
codgen_model = 'true';
nlp_solver = 'sqp_rti';
% nlp_solver_max_iter = 100;
qp_solver = 'partial_condensing_hpipm';
nlp_solver_exact_hessian = 'false';
qp_solver_cond_N = 50;
% qp_solver_iter_max = 40;
regularize_method = 'no_regularize'; % regularize_method = 'convexify';
sim_method = 'erk'; % integrator type
qp_solver_warm_start = 1;
nlp_solver_warm_start_first_qp = 1;
print_level = 0;
sim_method_num_stages = 4; % RungeKutta4
sim_method_num_steps = 3;
nlp_solver_tol_stat = 5e-3;
nlp_solver_tol_eq = 5e-3;
nlp_solver_tol_ineq = 5e-3;
nlp_solver_tol_comp = 5e-3;

%% Dynamics
if strcmp(sim_method, 'erk')
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.f_expl_expr);
else
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.f_impl_expr);
end

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
settings = struct();

% Set each parameter only if it exists, both in acados options and settings struct.
if exist('compile_interface', 'var')
    settings.compile_interface = compile_interface;
    ocp_opts.set('compile_interface', compile_interface);
end
if exist('codgen_model', 'var')
    settings.codgen_model = codgen_model;
    ocp_opts.set('codgen_model', codgen_model);
end
if exist('nlp_solver', 'var')
    settings.nlp_solver = nlp_solver;
    ocp_opts.set('nlp_solver', nlp_solver);
end
if exist('qp_solver', 'var')
    settings.qp_solver = qp_solver;
    ocp_opts.set('qp_solver', qp_solver);
end
if exist('nlp_solver_exact_hessian', 'var')
    settings.nlp_solver_exact_hessian = nlp_solver_exact_hessian;
    ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
end
if exist('qp_solver_cond_N', 'var')
    settings.qp_solver_cond_N = qp_solver_cond_N;
    ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
end
if exist('regularize_method', 'var')
    settings.regularize_method = regularize_method;
    ocp_opts.set('regularize_method', regularize_method);
end
if exist('sim_method', 'var')
    settings.sim_method = sim_method;
    ocp_opts.set('sim_method', sim_method);
end
if exist('qp_solver_warm_start', 'var')
    settings.qp_solver_warm_start = qp_solver_warm_start;
    ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
end
if exist('print_level', 'var')
    settings.print_level = print_level;
    ocp_opts.set('print_level', print_level);
end
if exist('sim_method_num_stages', 'var')
    settings.sim_method_num_stages = sim_method_num_stages;
    ocp_opts.set('sim_method_num_stages', sim_method_num_stages);
end
if exist('sim_method_num_steps', 'var')
    settings.sim_method_num_steps = sim_method_num_steps;
    ocp_opts.set('sim_method_num_steps', sim_method_num_steps);
end
if exist('nlp_solver_tol_stat', 'var')
    settings.nlp_solver_tol_stat = nlp_solver_tol_stat;
    ocp_opts.set('nlp_solver_tol_stat', nlp_solver_tol_stat);
end
if exist('nlp_solver_tol_eq', 'var')
    settings.nlp_solver_tol_eq = nlp_solver_tol_eq;
    ocp_opts.set('nlp_solver_tol_eq', nlp_solver_tol_eq);
end
if exist('nlp_solver_tol_ineq', 'var')
    settings.nlp_solver_tol_ineq = nlp_solver_tol_ineq;
    ocp_opts.set('nlp_solver_tol_ineq', nlp_solver_tol_ineq);
end
if exist('nlp_solver_tol_comp', 'var')
    settings.nlp_solver_tol_comp = nlp_solver_tol_comp;
    ocp_opts.set('nlp_solver_tol_comp', nlp_solver_tol_comp);
end
if exist('nlp_solver_max_iter', 'var')
    settings.nlp_solver_max_iter = nlp_solver_max_iter;
    ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
if exist('qp_solver_iter_max', 'var')
    settings.qp_solver_iter_max = qp_solver_iter_max;
    ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
end
if exist('nlp_solver_warm_start_first_qp', 'var')
    settings.warm_start_first_qp = nlp_solver_warm_start_first_qp;
    ocp_opts.set('nlp_solver_warm_start_first_qp', nlp_solver_warm_start_first_qp);
end

% Set param_scheme_N separately as it is always defined
settings.param_scheme_N = N;
ocp_opts.set('param_scheme_N', N);

end
