function Ll2f = Ll2f_gen(x,y,theta,V,R,xo,yo)
%LL2F_GEN
%    LL2F = LL2F_GEN(X,Y,THETA,V,R,XO,YO)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    10-Aug-2020 22:06:20

t2 = cos(theta);
t3 = sin(theta);
Ll2f = V.*t2.*(x.*2.0-xo.*2.0+R.*t3.*2.0)-V.*t3.*(y.*-2.0+yo.*2.0+R.*t2.*2.0);
