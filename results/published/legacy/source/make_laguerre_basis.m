function basis = make_laguerre_basis(horizon, order, alpha)
validateattributes(horizon, {'numeric'}, {'scalar','integer','positive'});
validateattributes(order, {'numeric'}, {'scalar','integer','positive','<=',horizon});
validateattributes(alpha, {'numeric'}, {'scalar','real','finite','>',0,'<',1});
basis = zeros(horizon, order);
basis(:,1) = sqrt(1-alpha^2)*alpha.^(0:horizon-1)';
for column = 2:order
    for k = 1:horizon
        if k == 1
            basis(k,column) = -alpha*basis(k,column-1);
        else
            basis(k,column) = alpha*basis(k-1,column) + ...
                basis(k-1,column-1) - alpha*basis(k,column-1);
        end
    end
end
end
