function D = compute_drag(v, Cd)

A = 0.00353; % Surface area in m^2
rho = 1.44; % Atmospheric density (using sea level for now) kg/m^3

D = -0.5*Cd*rho*A*(v^2);

end
