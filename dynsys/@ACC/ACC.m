%% Main Reference
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

classdef ACC < CtrlAffineSys    
    methods        
        function obj = ACC(params)
            obj = obj@CtrlAffineSys(params);            
        end
        function Fr = getFr(obj, s)
            v = s(2);
            Fr = obj.params.f0 + obj.params.f1 * v + obj.params.f2 * v^2;
        end
    end
end
