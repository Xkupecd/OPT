function [theta_hat,t_hat,H] = transform_estimate2D(pA,pB,theta_hat,t_hat,tol,Lambda)

[x_hat,~,H] = optim_GaussNewton(@(x_hat)get_rJ(x_hat,pA,pB),[theta_hat;t_hat],tol,Lambda);

theta_hat = x_hat(1); %TODO
t_hat     = x_hat(2:3); %TODO

end


function [r,J] = get_rJ(x_hat,pA,pB)


theta_hat = x_hat(1); %TODO
t_hat     = x_hat(2:3); %TODO

N=size(pA,1);

R_hat   = rotZ(theta_hat);   R_hat   = R_hat(1:2,1:2); %TODO
d_R_hat = drotZ(theta_hat);  d_R_hat = d_R_hat(1:2,1:2); %TODO

r = reshape(R_hat*pA' + t_hat - pB', 2*N, 1);   % rezidua, 2N x 1 %TODO
df_d_theta = reshape(d_R_hat*pA', 2*N, 1);      % 2N x 1 %TODO
J = [df_d_theta, repmat(eye(2), N, 1)];         % 2N x 3 %TODO

end