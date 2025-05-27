function dist_vector = dist_circles(x0_ego, h, model)

% Cars lengths
D1 = model.params.D1;
D2 = model.params.D2;
D1_h = model.params.D1_h;
D2_h = model.params.D2_h;

% Current states 
s0 = x0_ego(1);
y0 = x0_ego(2);
theta0 = x0_ego(3);
shdv0 = h.simX_0(1);
yhdv0 = h.simX_0(2);
thetahdv0 = h.simX_0(3);

% Distances of the 4 circles
dist1 = (s0+D1/4*cos(theta0)-(shdv0+D1_h/4*cos(thetahdv0)))^2 ...
    +(y0+D1/4*sin(theta0)-(yhdv0+D1_h/4*sin(thetahdv0)))^2 ...
    -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;

dist2 = (s0+D1/4*cos(theta0)-(shdv0-D1_h/4*cos(thetahdv0)))^2 ...
    +(y0+D1/4*sin(theta0)-(yhdv0-D1_h/4*sin(thetahdv0)))^2 ...
    -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;

dist3 = (s0-D1/4*cos(theta0)-(shdv0+D1_h/4*cos(thetahdv0)))^2 ...
    +(y0-D1/4*sin(theta0)-(yhdv0+D1_h/4*sin(thetahdv0)))^2 ...
    -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;

dist4 = (s0-D1/4*cos(theta0)-(shdv0-D1_h/4*cos(thetahdv0)))^2 ...
    +(y0-D1/4*sin(theta0)-(yhdv0-D1_h/4*sin(thetahdv0)))^2 ...
    -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;

% Regroup vector
dist_vector = [dist1, dist2, dist3, dist4];

end
