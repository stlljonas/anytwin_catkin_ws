function mat_out = productTensorMat(T,mat_in,dim)
[a,b,c] = size(T);
[n,m] = size(mat_in);

if dim == 2
    mat_out = sym(zeros(a,m,c));
elseif dim == 3
    mat_out = sym(zeros(a,m,b));
end

for k=1:n
    if dim == 2
        mat_out(:,:,k) = T(:,:,k)*mat_in;
    elseif dim == 3
        % todo
    end
end

end