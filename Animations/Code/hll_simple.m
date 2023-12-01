clear all
clc

rho = 0.0023769;
omega = 100;
liftslope = 5.9;
cd0 = 0.02;

num_as = 100;

radius = 1;
n_blades = 1;
rc = 0.1;

r = linspace(0.5 * (1 / num_as), 1 - 0.5 * (1 / num_as), num_as);
psi = 30;

% freestream in the x-direction
fs_x = 30;

% Tangent velocities to each airstation
v = fs_x * sind(psi) + omega .* r;

inflow = -10;
phi = atan(-inflow ./ v);

theta = 0.1;
aoa = theta - phi;

as_area = rc * 1 / num_as;

lift = zeros(1, length(r));

for i = 1:length(r)
    if v(i) > 0
        lift(i) = 0.5 * (liftslope * aoa(i)) * rho .* (v(i)^2 + inflow^2) * as_area;
    else
        lift(i) = -0.5 * (liftslope * aoa(i)) * rho .* (v(i)^2 + inflow^2) * as_area;
    end
end

% Rotates back into body frame
fz = lift .* cos(phi);

total_fz = sum(fz);


fx = lift .* sin(phi);

moment = -fx .* r;

total_moment = sum(moment);

ct = total_fz / (rho * pi * radius^2 * (omega * radius)^2);

% Currently does not take drag into account. See C++ results for more
% accurate comparison to theory
cq = -total_moment / (rho * pi * radius^2 * (omega * radius)^2 * radius);


sigma = n_blades * rc / (pi * radius);
lambda = -inflow / (omega * radius);
mu = fs_x / (omega * radius);



ct_theory = 0.5 * sigma * liftslope * ((1/3 + mu * sind(psi) + mu^2 * sind(psi)^2) * theta - 0.5 * lambda - lambda * mu * sind(psi));
cq_theory = sigma * liftslope * lambda * (theta / 6 + mu * theta / 4 * sind(psi) - lambda / 4) + sigma * cd0 / 2 * (1/4 + 2/3 * mu * sind(psi) + mu^2 * sind(psi)^2);


ct_err = (ct - ct_theory) / ct_theory
cq_err = (cq - cq_theory) / cq_theory




% Not fully understood
fz_johnson = liftslope .* rc .* (v.^2 + inflow^2) .* (0.5 .* aoa .* cos(phi));

fx_johnson = liftslope .* rc .* (v.^2 + inflow^2) .* (0.5 .* aoa .* sin(phi));



%% Plotting the integrand of the lift forces and the actual data
figure (1);
hold on

plot(r, fz ./ (radius / num_as))

integrand_ct = @(r)(0.5 .* sigma .* liftslope * ((r + mu * sind(psi)).^2 .* theta - lambda .* (r + mu .* sind(psi))));
plot(r, integrand_ct(r) .* rho .* (pi .* radius^2) .* (omega .* radius).^2)

xlabel('Radial Distance')
ylabel('F_Z / R')
title('F_Z / R Across Blade')

legend({'Hermes', 'Theory'}, 'Location', 'Southeast')



