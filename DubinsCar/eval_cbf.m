function [h, Lfh, Lgh, left_active] = eval_cbf(s, params)
    x = s(1);
    y = s(2);
    theta = s(3);
    V = params.V;
    R1 = params.R_left;
    R2 = params.R_right;
    xo = params.xo; yo = params.yo; d = params.d;
    
    l1 = l1_gen(x, y, theta, params.R_left, params.xo, params.yo, params.d);
    l2 = l2_gen(x, y, theta, params.R_right, params.xo, params.yo, params.d);
%     
% %     if l1 < l2
%         h = l1;
%         Lfh = Ll1f_gen(x, y, theta, params.V, params.R_left, params.xo, params.yo);
%         Lgh = Ll1g_gen(x, y, theta, params.R_left, params.xo, params.yo);
%         left_active = 1;
%     else
%         h = l2;
%         Lfh = Ll2f_gen(x, y, theta, params.V, params.R_right, params.xo, params.yo);
%         Lgh = Ll2g_gen(x, y, theta, params.R_right, params.xo, params.yo);        
%         left_active = 0;
%     end



    h = l12_gen(x, y, theta, R1, R2, xo, yo, d);
    Lfh = Lfl12_gen(x, y, theta, V, R1, R2, xo, yo, d);
    Lgh = Lgl12_gen(x, y, theta, V, R1, R2, xo, yo, d);

end