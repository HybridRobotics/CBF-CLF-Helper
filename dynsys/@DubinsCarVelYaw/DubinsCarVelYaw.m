classdef DubinsCarVelYaw < CtrlAffineSys
    methods
        function [s, f, g] = defineSystem(obj, params)
            syms x y theta
            s = [x; y; theta];
            f = [0;0;0];
            g = [cos(theta), 0; sin(theta), 0; 0, 1];
        end
        
        function clf = defineClf(~, params, symbolic_s)
            x = symbolic_s(1);
            y = symbolic_s(2);
            theta = symbolic_s(3);
            clf = (cos(theta).*(y-params.yd)-sin(theta).*(x-params.xd)).^2;
        end
        
        function cbf = defineCbf(~, params, symbolic_s)
            s = symbolic_s; 
            x = s(1); y = s(2); theta = s(3);

            xo = params.xo;
            yo = params.yo;
            d = params.d;
            b = params.b;
            a1 = params.a1; a2 = params.a2; a3 = params.a3;
            R_left = params.u_min(1) / params.u_max(2);
            R_right = - params.u_min(1) / params.u_min(2);

            % Left CBF
%             cbf = (x - xo - R_left * sin(theta))^2 + (y - yo + R_left * cos(theta))^2 - (R_left + d)^2;

%             n = (x-xo) * cos(theta) + (y-yo) * sin(theta);
            distance = (x - xo)^2 + (y - yo)^2 - b * d^2;
%             cbf = distance + a1 * atan(a2 * n + a3); 
            cbf = distance - 0.5 * theta ^2;
        end
    end 
end