function ds = dynamics(t, s, u, params)

ds = zeros(3, 1);

ds(1) = params.V * cos(s(3));
ds(2) = params.V * sin(s(3));
ds(3) = u;