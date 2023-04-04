function S = skewMat(mat)
    [~,b]= size(mat);
    S = zeros(3,3,b);
    for k=1:b
        S(:,:,k) = skew(mat(:,k));
    end
end