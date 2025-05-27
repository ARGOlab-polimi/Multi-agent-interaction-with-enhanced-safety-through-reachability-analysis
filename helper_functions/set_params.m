function params = set_params(h,u0,i)

Mhj_0 = [h.p(i,5); h.p(i,6)];
grad_V = h.p(i,:)';

bhj_0 = grad_V'*h.rel_dyn(i,:)'-Mhj_0'*u0;


if isnan(h.V(i))

    error('is NaN')
else
    
    params = [Mhj_0; bhj_0; h.settings.onoff(i); h.simX_0(1:3)'];
end
