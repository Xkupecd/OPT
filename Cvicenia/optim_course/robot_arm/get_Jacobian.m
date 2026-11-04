function [J] = get_Jacobian(theta, pn, R, dR, T)

J = zeros(3, length(R));

for i = 1:length(R)

    % dp0/dtheta_i: od vnutra von, v clanku i sa R nahradi deriváciou dR
    p  = pn;          % poloha bodu v ramci j (hodnota)
    dp = zeros(3,1);  % jej derivacia podla theta_i
    for j = length(R):-1:1
        if j == i
            dp = dR{j}(theta(j)) * (T(:,j) + p);
        else
            dp = R{j}(theta(j)) * dp;
        end
        p = R{j}(theta(j)) * (T(:,j) + p);
    end
    J(:, i) = dp;

end

end
