classdef DubinsCar < CtrlAffineSys
    methods
        function [x, f, g] = defineSystem(obj, params)
            syms p_x p_y theta
            x = [p_x; p_y; theta];
            f = [params.v * cos(theta);
                params.v * sin(theta);
                0];
            g = [0; 0; 1];
        end
    end 
end