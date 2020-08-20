classdef DubinsCar < CtrlAffineSys
    methods
        function [s, f, g] = defineSystem(obj, params)
            syms x y theta
            s = [x; y; theta];
            f = [params.v * cos(theta);
                params.v * sin(theta);
                0];
            g = [0; 0; 1];
        end
    end 
end