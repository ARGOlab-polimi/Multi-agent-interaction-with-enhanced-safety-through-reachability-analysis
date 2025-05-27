function h = human_control(h,i,model,dt)

lr = model.params.lr;
lf = model.params.lf;

delta_max = model.delta_max;

h.simU_0 = [0;0];

if h.settings.do_hdv_control

    w_h = h.settings.wh_coeff*h.simX_0(4)*cos(atan(lr/(lf+lr)*tan(delta_max))) ...
        /(lr+lf)*tan(delta_max);

    if h.p(i,3) > h.settings.p_threshold
        h.simU_0(1) = -w_h;
    elseif h.p(i,3) < -h.settings.p_threshold
        h.simU_0(1) = w_h;
    end

    if h.p(i,4) < -h.settings.p_threshold

        if h.simX_0(4)+model.dv_max*dt <= h.g.max(4)
            h.simU_0(2) = model.dv_max;
        else
            h.simU_0(2) = (h.g.max(4)-h.simX_0(4))/dt;
        end

    elseif h.p(i,4) > h.settings.p_threshold

        if h.simX_0(4)+model.dv_min*dt < h.g.min(4)
            h.simU_0(2) = (h.g.min(4)-h.simX_0(4))/dt;
        else
            h.simU_0(2) = model.dv_min;
        end

    end

    % Additional penalty term

    if h.settings.add_penalty
        h.simU_0(1) = h.simU_0(1)*0.5;
        h.simU_0(2) = h.simU_0(2)*0.5;
    end

end
