function U_safe = hamiltonian(h,u0_new)

U_safe = h.params(1:2)'*u0_new+h.params(3);



