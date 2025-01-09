function dxdt = SystemProblem01(t, x, para)
    % Control law
    u = ControlNonLinear(x, t, para);

    % System equations
    dxdt = para.A * x + para.B * u;
end
