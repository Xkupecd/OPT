function [theta] = solve_IK(pr, theta, R, dR, T, tol,Lambda)



while true
    p0 = transform_pi_to_p0(theta, [0;0;0], length(R), R, T); %TODO
    r  = p0 - pr; %TODO
    J = get_Jacobian(theta, [0;0;0], R, dR, T); % 3x6
   
    delta_theta = -(J'*J + Lambda) \ (J'*r); %TODO;
    theta = theta + delta_theta; %TODO;

    if (r' * r) < tol
        break
    end
end

% Wrap joint angles to [0, 2*pi]
theta = mod(theta, 2*pi);

end