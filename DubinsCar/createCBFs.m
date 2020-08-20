syms V R1 R2
syms x y theta % state
syms u % control
syms xo yo % obstacle center
syms d % obstacle radius

s = [x; y; theta];

f = [V * cos(theta);
    V * sin(theta);
    0];

g = [0; 0; 1];

l1 = (x - xo - R1 * sin(theta))^2 + (y - yo + R1 * cos(theta))^2 - (R1 + d)^2;
l2 = (x - xo + R2 * sin(theta))^2 + (y - yo - R2 * cos(theta))^2 - (R2 + d)^2;
l12 = l1 * l2;

dl1 = simplify(jacobian(l1, s));
dl2 = simplify(jacobian(l2, s));
dl12 = simplify(jacobian(l12, s));

Ll1f = dl1 * f;
Ll1g = dl1 * g;
Ll2f = dl2 * f;
Ll2g = dl2 * g;

Lfl12 = dl12 * f;
Lgl12 = dl12 * g;


% matlabFunction(l1, 'file', 'l1_gen.m', 'vars', [x, y, theta, R, xo, yo, d]);
% matlabFunction(Ll1f, 'file', 'Ll1f_gen.m', 'vars', [x, y, theta, V, R, xo, yo]);
% matlabFunction(Ll1g, 'file', 'Ll1g_gen.m', 'vars', [x, y, theta, R, xo, yo]);
% 
% matlabFunction(l2, 'file', 'l2_gen.m', 'vars', [x, y, theta, R, xo, yo, d]);
% matlabFunction(Ll2f, 'file', 'Ll2f_gen.m', 'vars', [x, y, theta, V, R, xo, yo]);
% matlabFunction(Ll2g, 'file', 'Ll2g_gen.m', 'vars', [x, y, theta, R, xo, yo]);

matlabFunction(l12, 'file', 'l12_gen.m', 'vars', [x, y, theta, R1, R2, xo, yo, d]);
matlabFunction(Lfl12, 'file', 'Lfl12_gen.m', 'vars', [x, y, theta, V, R1, R2, xo, yo, d]);
matlabFunction(Lgl12, 'file', 'Lgl12_gen.m', 'vars', [x, y, theta, V, R1, R2, xo, yo, d]);
