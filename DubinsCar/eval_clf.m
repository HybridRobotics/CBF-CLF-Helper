function [V, LgV] = eval_clf(s, params)

x = s(1);
y = s(2);
theta = s(3);

V = lyapFun(x, y, theta, params.xd, params.yd);
LgV = LgLyap(x, y, theta, params.xd, params.yd);