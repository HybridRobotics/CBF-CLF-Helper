function [func, LfFunc, LgFunc, dFunc] = evalFuncFromTable(s, grid, funcTable, f, g, dFuncTable)
    if nargin < 6
        dFuncTable = computeGradients(grid, funcTable);
    end
    func = eval_u(grid, funcTable, s);
    dFunc = eval_u(grid, dFuncTable, s);
    LfFunc = dFunc' * f(s);
    LgFunc = dFunc' * g(s);
end