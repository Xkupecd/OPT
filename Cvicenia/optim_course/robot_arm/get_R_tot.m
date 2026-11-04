function [R_tot] = get_R_tot(theta, R)

R_tot = eye(3,3);

for i = 1:length(R)
    R_tot = R_tot * R{i}(theta(i));
end

end
