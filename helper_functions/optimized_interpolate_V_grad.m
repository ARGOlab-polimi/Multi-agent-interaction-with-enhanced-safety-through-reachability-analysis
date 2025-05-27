function h = optimized_interpolate_V_grad(h,i)

if isempty(h.closest_point_idx)

    % If it is inside the grid, interpolate 

    grid_vectors = {h.x{1}(:), h.x{2}(:), h.x{3}(:), h.x{4}(:), h.x{5}(:), h.x{6}(:)};

    F_V = griddedInterpolant(grid_vectors, h.V_matrix);
    F_p1 = griddedInterpolant(grid_vectors, h.grad{1});
    F_p2 = griddedInterpolant(grid_vectors, h.grad{2});
    F_p3 = griddedInterpolant(grid_vectors, h.grad{3});
    F_p4 = griddedInterpolant(grid_vectors, h.grad{4});
    F_p5 = griddedInterpolant(grid_vectors, h.grad{5});
    F_p6 = griddedInterpolant(grid_vectors, h.grad{6});

    % Interpolate using the precomputed interpolants
    
    X_rel = h.rel_states(i,:);
    h.V(i) = F_V(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,1) = F_p1(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,2) = F_p2(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,3) = F_p3(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,4) = F_p4(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,5) = F_p5(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));
    h.p(i,6) = F_p6(X_rel(1), X_rel(2),X_rel(3),X_rel(4),X_rel(5),X_rel(6));

else

    % Compuct exact values of the closest point of the grid to the out of
    % grid relative state 

    h.V(i) = 1;
end