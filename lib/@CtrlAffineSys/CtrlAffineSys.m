classdef CtrlAffineSys < handle    
    properties
        sdim % state dimension
        udim % control input dimension dimension
        params % Model parameters
        f
        g
        cbf
        lf_cbf
        lg_cbf
        clf
        lf_clf
        lg_clf
    end
    
    methods
        function obj = CtrlAffineSys(params)
            if nargin == 0
                obj.params = [];
                disp("Warning: params argument is missing.")
            end
            obj.params = params;
            %% TODO: Add checking input constraint.
            %% TODO: Add existence of essential fields (e.g. params.weight.slack)
            [s, f, g] = obj.defineSystem(params);
            clf = obj.defineClf(params, s);
            cbf = obj.defineCbf(params, s);
            obj.initSys(s, f, g, cbf, clf);
        end
        %% Defining class functions.
        function [s, f, g] = defineSystem(obj, params)
            s = []; f = []; g = [];         
        end
        
        function clf = defineClf(obj, params, symbolic_state)
            clf = [];
        end
        
        function cbf = defineCbf(obj, params, symbolic_state)
            cbf = [];
        end
                
        function ds = dynamics(obj, t, s, u)
            ds = obj.f(s) + obj.g(s) * u;
        end
    end
end

