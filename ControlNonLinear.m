function u = ControlNonLinear(x, t, para)
    % Output y based on system state
    y = para.C * [x(1) x(2) x(3)]';
    
    omega = 0.56;
    amplitude = 2.16;
    y = y - amplitude*sin(omega*t);

    % Ensure y is valid for computation
    if abs(y) > 0
        u = real(2 / (pi * y) * exp(-1i * asin(2 / y)));
    else
        u = 0; % Handle edge case for y = 0
    end
end