function [x_hat, g, H] = optim_GaussNewton(f, x_hat, tol,Lambda)

while 1
    [r, J] = f(x_hat);
    H = J' * J + Lambda; %TODO  ;
   
    g = J' * r; %TODO
   

    if g' * g < tol
        break;
    end

    delta = H \ g; %TODO
    x_hat = x_hat - delta; %TODO

 
end

end