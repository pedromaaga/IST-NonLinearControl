function u = ControlLawIOFL(x, K, v)
    % Control law
    u = -x(2) -x(1)*x(3) + K*x(2) + v;
    
end

