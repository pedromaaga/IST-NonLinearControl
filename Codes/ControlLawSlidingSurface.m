function u = ControlLawSlidingSurface(x, Phi, eta, rho, K)

    s = x(1) + x(2) + x(3);
    % Control law
    if abs(s) <= Phi
        u = (-x(1) -x(2) -x(1)*x(3))/(2) - (rho+eta)*(s/Phi);
    else
        u = K(1)*x(1) + K(2)*x(2) + K(3)*x(3) - (rho+eta)*sign(s);
    end    
    
end

