function S = skewMatSym(mat)
    [~,b]= size(mat);
    S = sym(zeros(3,3,b));
    for k=1:b
        S(:,:,k) = skew(mat(:,k));
    end
end