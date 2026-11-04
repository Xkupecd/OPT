function [theta_hat, t_hat] = ICP_estimate_2D(pA, pB, theta_hat, t_hat, tol, tol_d, KDtree,Lambda)

i=0;
    while true
        % 2D rotation matrix from theta_hat (single angle)
        % R_hat = rotZ(theta_hat); %TODO
        R_hat = rotZ(theta_hat);  R_hat = R_hat(1:2,1:2);

        % Transform points pA using current estimate
        pA_transformed = (R_hat*pA' + t_hat)'; %TODO
        
        % Find nearest neighbors and distances using knnsearch
        [idx, d] = knnsearch(KDtree, pA_transformed,'k',1);

        % Estimate updated transform parameters with correspondences
        [theta_hat, t_hat, ~] = transform_estimate2D(pA, pB(idx,:), theta_hat, t_hat, tol, Lambda); %TODO

        % Check convergence by mean distance
        i=i+1;
        if mean(d) < tol_d || i>200
            break;
        end
    end

end