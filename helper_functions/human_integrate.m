function h = human_integrate(h,i,dt)

v_h = h.simX_0(4);

h.simX(i,:) = [
    v_h*cos(h.simX_0(3)),...
    v_h*sin(h.simX_0(3)),...
    h.simU_0(1),...
    h.simU_0(2)]*dt+h.simX_0; 

h.simX_0 = h.simX(i,:);
h.simU_0 = h.simU(i,:);