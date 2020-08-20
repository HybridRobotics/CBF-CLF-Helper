classdef DoubleIntegrator2D < CtrlAffineSys
    methods
        function clf = defineClf(obj, params, symbolic_s)
            s = symbolic_s;

            A = zeros(4);
            A(1, 2) = 1;
            A(3, 4) = 1;
            B = [0 0; 1 0; 0 0; 0 1];
            Q = eye(size(A));
            R = eye(size(B,2));
            [~,P] = lqr(A,B,Q,R);
            e = s - [params.p_d(1); 0; params.p_d(2); 0];
            clf = e' * P * e;        
        end
        
        function cbf = defineCbf(obj, params, symbolic_s)
            s = symbolic_s;
            p_o = params.p_o; % position of the obstacle.
            r_o = params.r_o; % radius of the obstacle.
            distance = (s(1) - p_o(1))^2 + (s(3) - p_o(2))^2 - r_o^2;
            derivDistance = 2*(s(1)-p_o(1))*s(2) + 2*(s(3)-p_o(2))*s(4);
            cbf = derivDistance + params.cbf_gamma0 * distance;
        end
    end    
end
