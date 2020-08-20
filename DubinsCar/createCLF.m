syms V
syms x y theta
syms u
syms xd yd % desired position.

s = [x; y; theta];

f = [V * cos(theta);
    V * sin(theta);
    0];

g = [0; 0; 1];

lyapFun = (-(xd-x)*sin(theta)+(yd-y)*cos(theta))^2;

dlyap = simplify(jacobian(lyapFun, s));

LfLyap = dlyap * f; % this is 0.
LgLyap = dlyap * g;

matlabFunction(lyapFun, 'file', 'lyapFun.m', 'vars', [x, y, theta, xd, yd]);
matlabFunction(LgLyap, 'file', 'LgLyap.m', 'vars', [x, y, theta, xd, yd]);