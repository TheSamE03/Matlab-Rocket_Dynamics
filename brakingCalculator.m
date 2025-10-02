function Cd = computeBrake(vz, z, targ, trim)
% Continuous Cd solver with latency-compensated state prediction.

% ---- params (tune as needed)
m=1.0; g=9.81; 
rho=1.225; 
S=0.00353;
Cmin=0.40; 
Cmax=1.60;
latency=0.12;          % measured actuation + compute delay
% trim   = 0.0;          % small bias (m), start at 0, then +/- 1–2 m if needed
% --------------------------------

% upward-only ascent logic
v = max(vz,0);
if v <= 1e-6
    Cd = Cmin; return;
end

% --- 1) Predict state through latency with brakes stowed (Cd = Cmin)
k0 = 0.5*rho*S*Cmin/m;
a0 = -(g + k0*v^2);                         % downward accel during latency
dt = latency;

z_pred = z + v*dt + 0.5*a0*dt^2;            % note: a0 is negative
v_pred = max(v + a0*dt, 0);                 % clamp to 0 if we flip before end of latency

% Remaining distance after latency
d_remaining = max(targ - z_pred, 0);
d_need = max(d_remaining - trim, 0);        % use small trim only if needed

% If we’re already past target (or no upward speed), stow
if d_need <= 1e-6 || v_pred <= 1e-6
    Cd = Cmin; return;
end

% --- 2) Stopping distance function from v_pred under g + k v^2
stopDist = @(Cd) (Cd<=1e-9).*(v_pred.^2/(2*g)) + ...
    (Cd>1e-9).* ( (1./(2*(0.5*rho*S*Cd/m))) .* log(1 + (0.5*rho*S*Cd/m)/g * v_pred.^2) );
% equals (1/(2k)) * ln(1 + (k/g) v^2), k = 0.5*rho*S*Cd/m

dCmin = stopDist(Cmin);
dCmax = stopDist(Cmax);    % NOTE: dCmax <= dCmin (more drag = shorter stop)

% Saturation guards
if dCmax > d_need
    % Even full brake can't shorten enough -> go max brake
    Cd = Cmax; return;
elseif dCmin < d_need
    % Even coasting is too short -> stow brakes
    Cd = Cmin; return;
end

% --- 3) Bisection on decreasing function g(Cd)=stopDist(Cd)-d_need
a = Cmin; b = Cmax;
for i=1:10
    c  = 0.5*(a+b);
    dc = stopDist(c);
    if dc > d_need
        a = c;           % need shorter distance -> increase Cd (more drag)
    else
        b = c;           % distance too short -> decrease Cd
    end
end
Cd = min(max(0.5*(a+b), Cmin), Cmax);
end
