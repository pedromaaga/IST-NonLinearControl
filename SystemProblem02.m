function dxdt = SystemProblem02(t, x, controlLaw)
    % Control law
    u = controlLaw(x);

    % Equações do sistema
    dx1dt = -x(1) + u;
    dx2dt = -x(2) - x(1)*x(3) + u;
    dx3dt = x(1)*x(2);
    
    % Just to have the control action
    u_prev = x(4);
    du_dt = (u - u_prev);

    dxdt = [dx1dt; dx2dt; dx3dt; du_dt];
end

