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
        grid
        clf_table
        dclf_table
        cbf_table
        dcbf_table
        use_clf_table
        use_cbf_table
    end
    
    methods
        function obj = CtrlAffineSys(params, grid, clf_table, cbf_table)
            if nargin < 1
                obj.params = [];
                disp("Warning: params argument is missing.")
            else
                obj.params = params;
            end
            
            if nargin < 2
                obj.grid = [];
            else
                obj.grid = grid;
            end
            
            if nargin < 3
                obj.clf_table = [];
            else
                obj.clf_table = clf_table;
            end
            if nargin < 4
                obj.cbf_table = [];
            else
                obj.cbf_table = cbf_table;
            end
            
            if isempty(obj.grid) || isempty(obj.clf_table)
                obj.use_clf_table = false;
            else
                %% TODO: add size check
                obj.use_clf_table = true;
                obj.dclf_table = computeGradients(grid, clf_table);
            end
            if isempty(obj.grid) || isempty(obj.cbf_table)
                obj.use_cbf_table = false;
            else
                %% TODO: add size check
                obj.use_cbf_table = true;
                obj.dcbf_table = computeGradients(grid, cbf_table);
            end                
                
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

