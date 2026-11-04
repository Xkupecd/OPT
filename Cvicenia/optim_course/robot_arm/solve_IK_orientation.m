function [theta] = solve_IK_orientation(pr, Rr, theta, R, dR, T, tol,Lambda)


while true
  
    p0 = transform_pi_to_p0(theta, [0;0;0], length(R), R, T);
    rp = p0 - pr; %TODO

    R_tot = get_R_tot(theta, R);
 
    Re = R_tot * Rr'; %TODO
    ro = [Re(2,1); Re(3,1); Re(3,2)]; %TODO
    Jp = get_Jacobian(theta, [0;0;0], R, dR, T);
   
    Jo = zeros(3, length(R));

    % v cykle cez i:
    for i = 1:length(R)
        dR_tot = get_dR_tot(theta, R, dR, i);
        
        M = dR_tot * Rr';

        Jo(:,i) = [M(2,1); M(3,1); M(3,2)]; %TODO
    end

    J = [Jp; Jo]; %TODO    
    delta_theta = -(J + Lambda) \ [rp; ro]; %TODO
    theta = theta + delta_theta; %TODO
    
    if (rp' * rp + ro' * ro) < tol
        break
    end
end

% Wrap joint angles to [0, 2*pi]
theta = mod(theta, 2*pi);


end