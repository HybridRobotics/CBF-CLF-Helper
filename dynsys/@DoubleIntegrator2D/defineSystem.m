function [x, f, g] = defineSystem(~, ~)
    syms p_x v_x p_y v_y;
    x = [p_x; v_x; p_y; v_y];

    A = zeros(4);
    A(1, 2) = 1;
    A(3, 4) = 1;
    B = [0 0; 1 0; 0 0; 0 1];

    f = A * x;
    g = B;
end